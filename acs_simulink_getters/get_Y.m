function [Y] = get_Y(q, dq_r, ddq_r)
    Y = [ddq_r, dq_r, sin(q)];
end