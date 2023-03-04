function [G] = get_G_one_dof()
    m = 1;
    g = -9.81;
    d = 2;

    G = m * g * d;
end