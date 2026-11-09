//! Port of `ExtLibs/ArduPilot/Fence.cs`
//!
//! (namespace `MissionPlanner.Utilities`)
//!
//! `DownloadFence` and `UploadFence` go through `mav_mission` and `MAVLinkInterface`, which
//! are not ported yet; they are added with those files.

use super::fencecircle::{CirclePolyType, FenceCircle};
use super::fencepolygon::{FencePolygon, PolygonPolyType};
use super::fencereturn::FenceReturn;
use crate::dotnet::Exception;
use crate::extlibs::mavlink::{MAV_CMD, MAV_FRAME};
use crate::extlibs::utilities::locationwp::Locationwp;
use crate::extlibs::utilities::pointlatlngalt::PointLatLngAlt;

/// An element of `List<object> Fences`: one of the three fence types.
#[derive(Debug, Clone, PartialEq)]
pub enum FenceItem {
    Circle(FenceCircle),
    Polygon(FencePolygon),
    Return(FenceReturn),
}

/// `public class Fence`
#[allow(non_snake_case)]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Fence {
    /// `public List<object> Fences`
    pub Fences: Vec<FenceItem>,
}

impl Fence {
    /// `LocationToFence(List<Locationwp> list)`: appends the fences the mission items describe
    /// and returns the whole list. A polygon is complete when it has as many vertices as its
    /// items' `p1`; an unfinished polygon is dropped.
    #[allow(non_snake_case)]
    pub fn LocationToFence(&mut self, list: &[Locationwp]) -> &Vec<FenceItem> {
        let mut pol = FencePolygon::default();

        for locationwp in list {
            let plla = PointLatLngAlt::from_locationwp(locationwp);
            let id = locationwp.id;
            if id == MAV_CMD::FENCE_POLYGON_VERTEX_EXCLUSION.0 as u16 {
                pol.Mode = PolygonPolyType::Exclusive;
                pol.Points.push(plla);

                if pol.Points.len() as f32 == locationwp.p1 {
                    self.Fences.push(FenceItem::Polygon(std::mem::take(&mut pol)));
                }
            } else if id == MAV_CMD::FENCE_POLYGON_VERTEX_INCLUSION.0 as u16 {
                pol.Mode = PolygonPolyType::Inclusive;
                pol.Points.push(plla);

                if pol.Points.len() as f32 == locationwp.p1 {
                    self.Fences.push(FenceItem::Polygon(std::mem::take(&mut pol)));
                }
            } else if id == MAV_CMD::FENCE_CIRCLE_EXCLUSION.0 as u16 {
                self.Fences.push(FenceItem::Circle(FenceCircle {
                    Mode: CirclePolyType::Exclusive,
                    Center: Some(plla),
                    Radius: locationwp.p1,
                }));
            } else if id == MAV_CMD::FENCE_CIRCLE_INCLUSION.0 as u16 {
                self.Fences.push(FenceItem::Circle(FenceCircle {
                    Mode: CirclePolyType::Inclusive,
                    Center: Some(plla),
                    Radius: locationwp.p1,
                }));
            } else if id == MAV_CMD::FENCE_HOME_CIRCLE_INCLUSION.0 as u16 {
                // centred on home; the item's lat/lng are not used
                self.Fences.push(FenceItem::Circle(FenceCircle {
                    Mode: CirclePolyType::HomeInclusive,
                    Center: None,
                    Radius: locationwp.p1,
                }));
            } else if id == MAV_CMD::FENCE_RETURN_POINT.0 as u16 {
                self.Fences.push(FenceItem::Return(FenceReturn { Return: Some(plla) }));
            }
        }

        &self.Fences
    }

    /// `FenceToLocation()`: the mission items for every fence, in order. A FenceReturn with no
    /// point throws NullReferenceException, as in C#.
    #[allow(non_snake_case)]
    pub fn FenceToLocation(&self) -> Result<Vec<Locationwp>, Exception> {
        let global = MAV_FRAME::GLOBAL.0 as u8;
        let mut out = vec![];
        for o in &self.Fences {
            match o {
                FenceItem::Circle(b) => {
                    out.push(Locationwp {
                        frame: global,
                        id: b.Mode.0 as u16,
                        lat: b.Center.as_ref().map(|c| c.Lat).unwrap_or(0.0),
                        lng: b.Center.as_ref().map(|c| c.Lng).unwrap_or(0.0),
                        p1: b.Radius,
                        ..Default::default()
                    });
                }
                FenceItem::Polygon(b) => {
                    for pt in &b.Points {
                        out.push(Locationwp {
                            frame: global,
                            id: b.Mode.0 as u16,
                            lat: pt.Lat,
                            lng: pt.Lng,
                            p1: b.Points.len() as f32,
                            ..Default::default()
                        });
                    }
                }
                FenceItem::Return(b) => {
                    let r = b
                        .Return
                        .as_ref()
                        .ok_or_else(|| Exception::NullReference("Object reference not set to an instance of an object.".into()))?;
                    out.push(Locationwp {
                        frame: global,
                        id: MAV_CMD::FENCE_RETURN_POINT.0 as u16,
                        lat: r.Lat,
                        lng: r.Lng,
                        ..Default::default()
                    });
                }
            }
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn item(id: MAV_CMD, p1: f32, lat: f64) -> Locationwp {
        Locationwp { id: id.0 as u16, p1, lat, lng: 1.0, ..Default::default() }
    }

    #[test]
    fn round_trip() {
        let items = vec![
            item(MAV_CMD::FENCE_RETURN_POINT, 0.0, 5.0),
            item(MAV_CMD::FENCE_POLYGON_VERTEX_INCLUSION, 3.0, 1.0),
            item(MAV_CMD::FENCE_POLYGON_VERTEX_INCLUSION, 3.0, 2.0),
            item(MAV_CMD::FENCE_POLYGON_VERTEX_INCLUSION, 3.0, 3.0),
            item(MAV_CMD::FENCE_CIRCLE_EXCLUSION, 50.0, 4.0),
        ];
        let mut f = Fence::default();
        assert_eq!(f.LocationToFence(&items).len(), 3);
        let back = f.FenceToLocation().unwrap();
        assert_eq!(back.len(), 5);
        assert_eq!(back[1].p1, 3.0);
        assert_eq!(back[4].id, MAV_CMD::FENCE_CIRCLE_EXCLUSION.0 as u16);
        assert!(Fence { Fences: vec![FenceItem::Return(FenceReturn::default())] }.FenceToLocation().is_err());
    }
}
