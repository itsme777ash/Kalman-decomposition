function [T,A_n,B_n,C_n] = Kalman_main(A,B,C)

n = size(A,1);
m = size(B,2);
p = size(C,1);

if(n<=0 || m<=0 || p<=0 || n<=max(m,p))
    disp("Invalid dimensions");
else
    %Finding Controllability matrix
    iter = A*B;
    con = [B iter];
    for i = 1:n-2
        iter = A*iter;
        con = [con iter];
    end

    %Finding observability matrix
    iter2 = C*A;
    obs = [C;iter2];
    for j = 1:n-2
        iter2 = iter2*A;
        obs = [obs;iter2];
    end

    reach = orth(con,1e-4);
    unobs = null(obs,1e-4);
    unreach = null(transpose(reach),1e-4);
    observ = null(transpose(unobs),1e-4);

    %Constructing intersection matrices
    S_cob = intersection(reach,unobs);
    S_co = intersection(reach,observ);
    S_cbob = intersection(unreach,unobs);
    S_cbo = intersection(unreach,observ);

    T = [S_cob S_co S_cbob S_cbo];  %Transformation matrix
    A_n = T\(A*T);
    B_n = T\B;
    C_n = C*T;

end

