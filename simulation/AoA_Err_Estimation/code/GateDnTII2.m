function  [D_na]=GateDnTII(h_ls_na,ind_hr11,ind_hr01,ind_hr10,ind_hr00,F0YX01,F0YX10,F0YX00,F0YX11,domi,delta,T)
%本函数用于生成累计分布并计算门限
%ind_hr=[ind_hr11,ind_hr01,ind_hr10,ind_hr00];
%基于量化hr的累计条件分布
F0YX11_na=PDF2(h_ls_na(ind_hr11),domi,delta,T);
F0YX01_na=PDF2(h_ls_na(ind_hr01),domi,delta,T);
F0YX10_na=PDF2(h_ls_na(ind_hr10),domi,delta,T);
F0YX00_na=PDF2(h_ls_na(ind_hr00),domi,delta,T);


F11=abs(F0YX11_na-F0YX11);
F01=abs(F0YX01_na-F0YX01);
F10=abs(F0YX10_na-F0YX10);
F00=abs(F0YX00_na-F0YX00);

D_na=(1/T)^2*sum(sum((1/4)*(F11+F10+F01+F00)));


