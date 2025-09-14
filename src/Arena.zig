const Self = @This();

data: []u8,

pub fn init(data: []u8) Self {
    return .{ .data = data };
}
