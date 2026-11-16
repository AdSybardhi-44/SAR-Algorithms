x = [
    22.5
    25
    27.5
    30
    32.5
    35
    37.5
    40
    42.5
    45
    47.5
    50];
y = [
    2703
    2971
    3212
    3452
    3720
    3961
    4202
    4469
    4710
    4951
    5219
    5460]/100;
reflector_offset = 1.3;
apc_offset = mean(y-x-reflector_offset)
apc_offset_variance = mean(mean((y-x-reflector_offset).^2) - apc_offset^2)
apc_offset_sigma_mm = sqrt(apc_offset_variance)*10