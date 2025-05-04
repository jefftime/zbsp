pub const v3 = struct {
    x: f32,
    y: f32,
    z: f32,

    pub fn norm(self: v3) v3 {
        _ = self;
        unreachable;
    }

    pub fn cross(self: v3, other: v3) v3 {
        _ = self;
        _ = other;
        unreachable;
    }
};
