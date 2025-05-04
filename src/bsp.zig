const std = @import("std");
const Map = @import("map.zig").Map;
const v3 = @import("geo.zig").v3;

pub const Bsp = struct {
    normal_id: u32 = 0,
    front: ?*Bsp = null,
    back: ?*Bsp = null,

    pub fn print_digraph(self: *const Bsp) void {
        _ = self;
        std.log.debug("Todo!", .{});
    }
};

const Poly = struct {
    face_id: u32,
};

const GeoData = struct {
    points: std.ArrayListUnmanaged(f32),
    faces: std.ArrayListUnmanaged(u32),
    normals: std.ArrayListUnmanaged(v3),
};

fn discretize(al: std.mem.Allocator, map: Map) !GeoData {
    _ = al;
    _ = map;
    return error.NotYet;
}

pub fn compile(al: std.mem.Allocator, map: Map) !Bsp {
    const data = try discretize(al, map);
    _ = data;

    return error.NotYet;
}
