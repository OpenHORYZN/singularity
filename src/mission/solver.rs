use anyhow::Context;
use argus_common::{GlobalPosition, LocalPosition, Waypoint};
use nalgebra::Vector3;
use px4_msgs::msg::{VehicleGlobalPosition, VehicleLocalPosition};

use crate::{
    trajectory::{Constraints3D, LowLevelWaypoint, TrajectoryController},
    util::{LocalPositionFeatures, VehicleLocalFeatures},
};

use super::StatefulMissionNode;

pub trait Materialize {
    fn materialize(
        self,
        local_position: &VehicleLocalPosition,
        global_height: f64,
    ) -> LocalPosition;
}

impl Materialize for Waypoint {
    fn materialize(
        self,
        local_position: &VehicleLocalPosition,
        global_height: f64,
    ) -> LocalPosition {
        match self {
            Waypoint::LocalOffset(o) => {
                let current = local_position.position();
                let offset: Vector3<f32> = o.cast();
                let offset: LocalPosition = offset.into();
                return offset + current;
            }
            Waypoint::GlobalFixedHeight {
                lat,
                lon,
                alt: height_amsl,
            } => LocalPosition::project(
                &GlobalPosition {
                    lat,
                    lon,
                    alt: height_amsl as f32,
                },
                &local_position,
            )
            .unwrap(),
            Waypoint::GlobalRelativeHeight {
                lat,
                lon,
                height_diff,
            } => LocalPosition::project(
                &GlobalPosition {
                    lat,
                    lon,
                    alt: (global_height + height_diff) as f32,
                },
                &local_position,
            )
            .unwrap(),
        }
    }
}

pub fn build_trajectory(
    local_position: VehicleLocalPosition,
    global_position: VehicleGlobalPosition,
    current_step: usize,
    plan: &[StatefulMissionNode],
    include_current_pos: bool,
    constraints: Constraints3D,
) -> Result<Option<TrajectoryController>, anyhow::Error> {
    let forward = plan.get(current_step..).context("out of bounds")?.iter();
    let mut waypoint_stack = vec![];

    if include_current_pos {
        waypoint_stack.push(LowLevelWaypoint::new(
            local_position.position().to_nalgebra().cast(),
        ))
    }

    for node in forward {
        match node {
            StatefulMissionNode::Takeoff { altitude } => {
                let current_wp = Waypoint::LocalOffset(Vector3::new(0.0, 0.0, 0.0));
                let takeoff_wp = Waypoint::LocalOffset(Vector3::new(0.0, 0.0, -altitude));
                let local_current =
                    current_wp.materialize(&local_position, global_position.alt.into());
                let local_takeoff =
                    takeoff_wp.materialize(&local_position, global_position.alt.into());
                waypoint_stack.push(LowLevelWaypoint::new(local_current.to_nalgebra().cast()));
                waypoint_stack.push(
                    LowLevelWaypoint::new(local_takeoff.to_nalgebra().cast()).with_constraints(
                        Constraints3D {
                            max_acceleration: Vector3::new(0.2, 0.2, 0.2),
                            ..constraints
                        },
                    ),
                )
            }
            StatefulMissionNode::Waypoint(wp) => {
                let local = if let Some(lp) = waypoint_stack.last() {
                    match wp {
                        Waypoint::LocalOffset(o) => (lp.position() + o).cast().into(),
                        Waypoint::GlobalFixedHeight { lat, lon, alt } => LocalPosition::project(
                            &GlobalPosition {
                                lat: *lat,
                                lon: *lon,
                                alt: *alt as f32,
                            },
                            &local_position,
                        )
                        .unwrap(),
                        Waypoint::GlobalRelativeHeight {
                            lat,
                            lon,
                            height_diff,
                        } => LocalPosition::project(
                            &GlobalPosition {
                                lat: *lat,
                                lon: *lon,
                                alt: *height_diff as f32 + global_position.alt,
                            },
                            &local_position,
                        )
                        .unwrap(),
                    }
                } else {
                    wp.clone().materialize(&local_position, 0.0)
                };

                waypoint_stack.push(LowLevelWaypoint::new(local.to_nalgebra().cast()));
            }
            StatefulMissionNode::Transition => {
                continue;
            }
            _ => break,
        }
    }

    if waypoint_stack.len() < 2 {
        return Ok(None);
    }

    let controller = TrajectoryController::new_constrained(waypoint_stack, constraints, 0.0)
        .context("could not create traj c")?;

    Ok(Some(controller))
}
