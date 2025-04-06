const std = @import("std");

pub const ScanOptions = packed struct(u4) {
    double_slash_comments: bool = false,
    hashtag_comments: bool = false,
    semicolon_comments: bool = false,
    scan_newlines: bool = false,
};

pub const TokenType = enum { ident, string, float, number, sym, unknown };
pub const Token = union(TokenType) {
    ident: []const u8,
    string: []const u8,
    float: []const u8,
    number: []const u8,
    sym: u8,
    unknown: u8,

    pub fn print(self: *const Token) void {
        switch (self.*) {
            .ident => |val| std.log.info("IDENT   : {s}", .{val}),
            .string => |val| std.log.info("STRING  : {s}", .{val}),
            .float => |val| std.log.info("FLOAT   : {s}", .{val}),
            .number => |val| std.log.info("NUMBER  : {s}", .{val}),
            .sym => |c| std.log.info("SYM     : {c}", .{c}),
            .unknown => |c| std.log.info("UNKNOWN : {}", .{c}),
        }
    }
};

const State = enum {
    readchar,
    newline,
    skip_whitespace,
    string,
    ident,
    number,
    sym,
    end,
};

const Scanner = struct {
    bufstart: [*]const u8,
    buf: []const u8,

    pub fn init(buf: []const u8) Scanner {
        std.log.info("buf.len: {}", .{buf.len});
        return .{
            .bufstart = buf.ptr,
            .buf = buf,
        };
    }

    pub fn pop(self: *Scanner) ?u8 {
        if (self.buf.len == 0) return null;

        const ret = self.buf[0];
        self.buf = self.buf[1..];
        return ret;
    }

    pub fn peek(self: *const Scanner) ?u8 {
        return self.peek_offset(0);
    }

    pub fn peek_offset(self: *const Scanner, offset: usize) ?u8 {
        if (self.buf.len == 0) return null;
        if (self.buf.len < offset) return null;

        return self.buf[offset];
    }

    pub fn pos(self: *const Scanner) usize {
        return @intFromPtr(self.buf.ptr) - @intFromPtr(self.bufstart);
    }

    pub fn slice(self: *const Scanner, start: usize, end: usize) []const u8 {
        return self.bufstart[start..end];
    }
};

pub fn scan_buf(
    al: std.mem.Allocator,
    buf: []const u8,
    comptime opts: ScanOptions, // TODO: Reconsider making this comptime
) !std.ArrayListUnmanaged(Token) {
    var tokens = try std.ArrayListUnmanaged(Token).initCapacity(al, 1024);
    var scanner = Scanner.init(buf);
    var curpos: usize = 0;
    var curtk: u8 = undefined;

    const isdigit = std.ascii.isDigit;
    const isspace = std.ascii.isWhitespace;
    const isalnum = std.ascii.isAlphanumeric;

    state: switch (State.readchar) {
        .readchar => {
            curpos = scanner.pos();
            curtk = scanner.pop() orelse continue :state .end;
            switch (curtk) {
                '\n', ' ', '\t', '\r' => {
                    if (opts.scan_newlines and curtk == '\n') {
                        continue :state .newline;
                    }

                    continue :state .skip_whitespace;
                },

                'a'...'z', 'A'...'Z' => continue :state .ident,

                '!'...'@', '['...'`', '{'...'~' => {
                    innersym: switch (curtk) {
                        '\"' => continue :state .string,
                        '_' => continue :state .ident,
                        '/' => {
                            if (!opts.double_slash_comments) break :innersym;
                            if (scanner.peek() != '/') break :innersym;
                            com: while (scanner.peek()) |ch| {
                                if (ch == '\n') break :com;
                                _ = scanner.pop();
                            }
                            continue :state .readchar;
                        },
                        '#' => {
                            if (!opts.hashtag_comments) break :innersym;
                            com: while (scanner.peek()) |ch| {
                                if (ch == '\n') break :com;
                                _ = scanner.pop();
                            }
                            continue :state .readchar;
                        },
                        ';' => {
                            if (!opts.semicolon_comments) break :innersym;
                            com: while (scanner.peek()) |ch| {
                                if (ch == '\n') break :com;
                                _ = scanner.pop();
                            }
                        },
                        '0'...'9' => continue :state .number,
                        '.' => {
                            const nexttk = scanner.peek() orelse 0;
                            if (isdigit(nexttk)) continue :state .number;
                        },
                        '-' => {
                            const nexttk = scanner.peek() orelse 0;
                            const nextnexttk = scanner.peek_offset(1) orelse 0;
                            if (isdigit(nexttk)) continue :state .number;
                            if (nexttk == '.' and isdigit(nextnexttk)) {
                                continue :state .number;
                            }
                        },
                        else => continue :state .sym,
                    }
                },

                else => {
                    try tokens.append(al, .{ .unknown = curtk });
                    continue :state .readchar;
                },
            }
        },

        .newline => {
            try tokens.append(al, .{ .sym = curtk });
            continue :state .readchar;
        },

        .skip_whitespace => {
            while (isspace(scanner.peek() orelse 0)) _ = scanner.pop();
            continue :state .readchar;
        },

        .ident => {
            ident: while (scanner.peek()) |ch| {
                if (isalnum(ch) or ch == '_') {
                    _ = scanner.pop();
                    continue :ident;
                }
                break :ident;
            }
            try tokens.append(al, .{
                .ident = scanner.slice(curpos, scanner.pos()),
            });
            continue :state .readchar;
        },

        .number => {
            var oneperiod = curtk == '.';
            var T: TokenType = if (curtk == '.') .float else .number;

            number: while (scanner.peek()) |ch| {
                if (ch == '.' and !oneperiod) {
                    oneperiod = true;
                    T = .float;
                    _ = scanner.pop();
                    continue :number;
                } else if (ch == '.' and oneperiod) {
                    break :number;
                }

                if (isdigit(ch)) {
                    _ = scanner.pop();
                    continue :number;
                }

                break :number;
            }

            var result: Token = undefined;
            switch (T) {
                .float => result = .{
                    .float = scanner.slice(curpos, scanner.pos()),
                },
                .number => result = .{
                    .number = scanner.slice(curpos, scanner.pos()),
                },
                // Unreachable?
                else => return error.NumberParse,
            }

            try tokens.append(al, result);
            continue :state .readchar;
        },

        .string => {
            string: while (scanner.peek()) |ch| {
                if (ch == '\"') break :string;
                _ = scanner.pop();
            }
            _ = scanner.pop();
            try tokens.append(al, .{
                .string = scanner.slice(curpos + 1, scanner.pos() - 1),
            });
            continue :state .readchar;
        },

        .sym => {
            try tokens.append(al, .{ .sym = curtk });
            continue :state .readchar;
        },

        .end => break :state,
    }

    return tokens;
}

test "scanner peek" {
    const input = "1234";
    var scanner = Scanner.init(input);

    try std.testing.expectEqual('1', scanner.peek());
}

test "scanner peek_offset" {
    const input = "1234";
    var scanner = Scanner.init(input);

    try std.testing.expectEqual('2', scanner.peek_offset(1));
}

test "scanner pop" {
    const input = "1234";
    var scanner = Scanner.init(input);

    try std.testing.expectEqual('1', scanner.pop());
    try std.testing.expectEqual('2', scanner.pop());
    try std.testing.expectEqual('3', scanner.pop());
    try std.testing.expectEqual('4', scanner.pop());
    try std.testing.expectEqual(null, scanner.pop());
}

test "scanner peek after pop" {
    const input = "1234";
    var scanner = Scanner.init(input);

    try std.testing.expectEqual('1', scanner.pop());
    try std.testing.expectEqual('2', scanner.peek());
}

test "single string parse" {
    const input =
        \\"yeah"
    ;

    const al = std.testing.allocator;
    var tokens = try scan_buf(al, input, .{});
    defer tokens.deinit(al);
    try std.testing.expectEqual(1, tokens.items.len);
    try std.testing.expectEqualStrings("yeah", tokens.items[0].string);
}

test "two string parse" {
    const input =
        \\ "yeah"
        \\ "no"
    ;

    const al = std.testing.allocator;
    var tokens = try scan_buf(al, input, .{});
    defer tokens.deinit(al);
    try std.testing.expectEqual(2, tokens.items.len);
    try std.testing.expectEqualStrings("yeah", tokens.items[0].string);
    try std.testing.expectEqualStrings("no", tokens.items[1].string);
}

test "empty string" {
    const input =
        \\ ""
    ;

    const al = std.testing.allocator;
    var tokens = try scan_buf(al, input, .{});
    defer tokens.deinit(al);
    try std.testing.expectEqual(1, tokens.items.len);
    try std.testing.expectEqualStrings("", tokens.items[0].string);
}

test "ignore newlines" {
    const input =
        \\
        \\
        \\ident
        \\
        \\
    ;

    const al = std.testing.allocator;
    var tokens = try scan_buf(al, input, .{ .scan_newlines = false });
    defer tokens.deinit(al);
    try std.testing.expectEqual(1, tokens.items.len);
    try std.testing.expectEqualStrings("ident", tokens.items[0].ident);
}

test "include newlines" {
    const input =
        \\ident
        \\
        \\ident2
    ;

    const al = std.testing.allocator;
    var tokens = try scan_buf(al, input, .{ .scan_newlines = true });
    defer tokens.deinit(al);
    try std.testing.expectEqual(4, tokens.items.len);
    try std.testing.expectEqualStrings("ident", tokens.items[0].ident);
    try std.testing.expectEqual('\n', tokens.items[1].sym);
    try std.testing.expectEqual('\n', tokens.items[2].sym);
    try std.testing.expectEqualStrings("ident2", tokens.items[3].ident);
}

test "number" {
    const input = "1234";
    const al = std.testing.allocator;
    var tokens = try scan_buf(al, input, .{});
    defer tokens.deinit(al);
    try std.testing.expectEqual(1, tokens.items.len);
    try std.testing.expectEqualStrings("1234", tokens.items[0].number);
}

test "negative number" {
    const input = "-1234";
    const al = std.testing.allocator;
    var tokens = try scan_buf(al, input, .{});
    defer tokens.deinit(al);
    try std.testing.expectEqual(1, tokens.items.len);
    try std.testing.expectEqualStrings("-1234", tokens.items[0].number);
}

test "float" {
    const input = "1.23";
    const al = std.testing.allocator;
    var tokens = try scan_buf(al, input, .{});
    defer tokens.deinit(al);
    try std.testing.expectEqual(1, tokens.items.len);
    try std.testing.expectEqualStrings("1.23", tokens.items[0].float);
}

test "negative float" {
    const input = "-1.23";
    const al = std.testing.allocator;
    std.log.debug("aweofij", .{});
    var tokens = try scan_buf(al, input, .{});
    defer tokens.deinit(al);
    try std.testing.expectEqual(1, tokens.items.len);
    try std.testing.expectEqualStrings("-1.23", tokens.items[0].float);
}

test "float leading period" {
    const input = ".123";
    const al = std.testing.allocator;
    var tokens = try scan_buf(al, input, .{});
    defer tokens.deinit(al);
    try std.testing.expectEqual(1, tokens.items.len);
    try std.testing.expectEqualStrings(".123", tokens.items[0].float);
}

test "two floats no spaces" {
    const input = "1.23.456";
    const al = std.testing.allocator;
    var tokens = try scan_buf(al, input, .{});
    defer tokens.deinit(al);
    try std.testing.expectEqual(2, tokens.items.len);
    try std.testing.expectEqualStrings("1.23", tokens.items[0].float);
    try std.testing.expectEqualStrings(".456", tokens.items[1].float);
}

test "negative float leading period" {
    const input = "-.123";
    const al = std.testing.allocator;
    var tokens = try scan_buf(al, input, .{});
    defer tokens.deinit(al);
    try std.testing.expectEqual(1, tokens.items.len);
    try std.testing.expectEqualStrings("-.123", tokens.items[0].float);
}
