function P = PDF2(input, domi, L, N)
    Num = length(input);
    P = zeros(N, N);
    if Num == 0
        return;  % 返回全零，避免除以0
    end
    for k = 1:Num
        if real(input(k)) < domi && real(input(k)) > -domi && ...
           imag(input(k)) < domi && imag(input(k)) > -domi
            i = floor((real(input(k)) + domi) / L) + 1;
            j = floor((imag(input(k)) + domi) / L) + 1;
            i = min(i, N);  % 防止边界溢出
            j = min(j, N);
            P(i,j) = P(i,j) + 1;
        end
    end
    P = P / Num;
end