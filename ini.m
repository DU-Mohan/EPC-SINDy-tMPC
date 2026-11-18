%% Simulation and base values
T_s=1e-5;
Tend=100;
T_island=0;

f_b=60;
w_b=2*pi*f_b;
V_b=230; % Line-to-line RMS voltage
S_b=1e3;
P_b=S_b; Q_b=S_b;
I_b=S_b/(sqrt(3)*V_b);
Z_b=V_b^2/S_b;
L_b=Z_b/w_b;
C_b=1/(w_b*Z_b);
V_m=sqrt(2/3)*V_b;
Vdc_n=3*V_m;

%% Common load settings (identification and control)
PF=0.95;
dPQload_max=100*S_b;
Pload0=0.4878*S_b;
Pload1=S_b/2; Qload1=S_b*sqrt(1-PF^2)/2;

%% Common inverter parameters
p0=struct();
p0.S_b=S_b;
p0.V_m=V_m;
p0.w_b=w_b;
p0.I_b=I_b;
p0.Vrng=2*V_m*[1,-1];
p0.Irng=3*I_b*[1,-1];

% Modes: 1 excitation, 2 tMPC, 3 PI, 4 RL, 5 Koopman, 6 SINDy.
p0.ctrl.mode=[0,1]; % [time, mode; time2, mode2; ...]
p0.ctrl.Ts=8.33e-3; % 1/120
p0.ctrl.T_dly=1e-4;
p0.ctrl.excit=[]; % [time, Ps, ws, Qs, Vs]

%% PMU
p0.pmu.dT=1e-4;
p0.pmu.Anos=1e-3*[V_b,V_b,V_b,I_b,I_b,I_b];
p0.pmu.seed=1234;
p0.pmu.pll.Kpi=[2/sqrt(2)*(2*pi*30),(2*pi*30)^2,0];
p0.pmu.pll.fc=100;
p0.pmu.pll.frng=[45,75];
p0.pmu.pll.dfrng=30;
p0.pmu=update_pmu(p0.pmu,T_s,[V_b,I_b],w_b);

%% GFM physical parameters
p=p0;
p.type="gfm";
p.gfm_type="droop"; 
p.RLC=[1e-2,10e-3,200e-6];
p.FPdroop=5e-2;
p.VQdroop=2e-2;
p.wc=5*2*pi;
p.Dp=0.14;
p.J=0.045;
p.Kpi.v=[5e-2,1];
p.tau=1e-4;
p.Kpi.i=p.RLC([2,1])/p.tau;
if p.gfm_type=="vsg"
    p.FPdroop=p.S_b/(p.w_b^2*p.Dp);
    p.wc=p.Dp/p.J;
end
p.kfp=p.FPdroop*p.w_b/p.S_b;
p.kvq=p.VQdroop*p.V_m/p.S_b;

%% GFM secondary control

p.ctrl.refs=[p.w_b;p.V_m]; 

p.tmpc.pIBR=struct('wc',p.wc,'kfp',p.kfp,'kvq',p.kvq, ...
    'Kpv',p.Kpi.v(1)/p.RLC(3),'Kiv',p.Kpi.v(2)/p.RLC(3));
p.tmpc.N=10;
p.tmpc.Q=1e4*diag([1,1,0]);
p.tmpc.R=diag([1,1e-2]);
p.tmpc.cX0=[p.w_b;p.V_m;0];
p.tmpc.rX=[p.w_b;p.V_m;inf];
p.tmpc.cU0=p.S_b*[0.5;0];
p.tmpc.rU=3*p.S_b*[1;1]; 
p.tmpc.Wmodel=[0.945;1.9105;0.00806];
p.tmpc.WQ=[706;842.5];
p.tmpc.pi=[0.2,10]; 

p.sindy=struct('trained',false,'Ts',p.ctrl.Ts);
p.koopman=struct('trained',false,'Ts',p.ctrl.Ts);
p.rl=struct('trained',false,'Ts',p.ctrl.Ts);
p.PI.Kp=[30;10]; % Frequency, voltage; 
p.PI.Ki=[1500;1000];
p=update_pIBR(p);
pGFM=p;

%% GFL physical parameters
p=p0;
p.type="gfl";
p.RLC=[5e-2,2e-3,20e-6];
p.FPdroop=5e-2;
p.VQdroop=5e-2;
p.Kpi.p=[0.1,10];
p.tau=2e-3;
p.Kpi.i=p.RLC([2,1])/p.tau;
p.kfp=p.FPdroop*p.w_b/p.S_b;
p.kvq=p.VQdroop*p.V_m/p.S_b;

p.pll.Kpi=[10,100,0];
p.pll.fc=100;
p.pll.frng=[48,66];
p.pll.dfrng=30;

%% GFL secondary control

p.ctrl.refs=[0.5;-0.2]*p.I_b; 

p.tmpc.pIBR=struct('Kp',p.Kpi.p(1)*p.I_b/p.S_b, ...
    'Ki',p.Kpi.p(2)*p.I_b/p.S_b, ...
    'kfp',p.kfp,'kvq',p.kvq,'tau',p.tau);
p.tmpc.N=10;
p.tmpc.Q=1e3*diag([1,1,0,0]);
p.tmpc.R=1e-3*eye(2);
p.tmpc.cX0=zeros(4,1);
p.tmpc.rX=3*[p.I_b;p.I_b;inf;inf];
p.tmpc.cU0=[p.S_b;p.S_b];
p.tmpc.rU=3*[p.S_b;p.S_b];
p.tmpc.Wmodel=[0.428;0.2375;0.8405;1.092];
p.tmpc.WQ=[120.5;6.62;151.5;3.243];
p.tmpc.dPQs_bar=30*S_b;
p.tmpc.pi=[5,1500;40,1500]; 

p.sindy=struct('trained',false,'Ts',p.ctrl.Ts);
p.koopman=struct('trained',false,'Ts',p.ctrl.Ts);
p.rl=struct('trained',false,'Ts',p.ctrl.Ts);
p.PI.Kp=[5;20]; % Frequency, voltage; 
p.PI.Ki=[1500;500];
p=update_pIBR(p);
pGFL=p;
clear p p0
