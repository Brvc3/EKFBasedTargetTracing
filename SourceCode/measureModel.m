function z = measureModel(state)
    angle = atan(state(3)/state(1));
    range = norm([state(1) state(3)]);
    z = [angle;range];
end