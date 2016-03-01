addpath('./matpower5.1');

%% define named indices into bus, gen, branch matrices
[PQ, PV, REF, NONE, BUS_I, BUS_TYPE, PD, QD, GS, BS, BUS_AREA, VM, ...
    VA, BASE_KV, ZONE, VMAX, VMIN, LAM_P, LAM_Q, MU_VMAX, MU_VMIN] = idx_bus;
[F_BUS, T_BUS, BR_R, BR_X, BR_B, RATE_A, RATE_B, RATE_C, ...
    TAP, SHIFT, BR_STATUS, PF, QF, PT, QT, MU_SF, MU_ST, ...
    ANGMIN, ANGMAX, MU_ANGMIN, MU_ANGMAX] = idx_brch;
[GEN_BUS, PG, QG, QMAX, QMIN, VG, MBASE, GEN_STATUS, PMAX, PMIN, ...
    MU_PMAX, MU_PMIN, MU_QMAX, MU_QMIN, PC1, PC2, QC1MIN, QC1MAX, ...
    QC2MIN, QC2MAX, RAMP_AGC, RAMP_10, RAMP_30, RAMP_Q, APF] = idx_gen;


casedata = 'case14'; %% default data file is 'case9.m'


%% read data
mpc = loadcase(casedata);

%% add zero columns to branch for flows if needed
if size(mpc.branch,2) < QT
  mpc.branch = [ mpc.branch zeros(size(mpc.branch, 1), QT-size(mpc.branch,2)) ];
end

%% convert to internal indexing
mpc = ext2int(mpc);
[baseMVA, bus, gen, branch] = deal(mpc.baseMVA, mpc.bus, mpc.gen, mpc.branch);

%% get bus index lists of each type of bus
[ref, pv, pq] = bustypes(bus, gen);

%% generator info
on = find(gen(:, GEN_STATUS) > 0);      %% which generators are on?
gbus = gen(on, GEN_BUS);                %% what buses are they at?


%% initial state
Va0 = bus(:, VA) * (pi/180);

%% build B matrices and phase shift injections
[B, Bf, Pbusinj, Pfinj] = makeBdc(baseMVA, bus, branch);

%% compute complex bus power injections (generation - load) 
%  adjusted for phase shifters and real shunts
Pbus = real(makeSbus(baseMVA, bus, gen)) - Pbusinj - bus(:, GS) / baseMVA;

Va = dcpf(B, Pbus, Va0, ref, pv, pq);


Va2 = dcpf(B, Pbus, Va, ref, pv, pq);

Va3 = dcpf(B, Pbus, Va2, ref, pv, pq);
