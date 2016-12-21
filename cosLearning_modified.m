%% CoS learning + sequence simulation 
%{
% CoS Learning simulation on a mobile robot doing a sequence
% File:          cosLearning.m
% Date:          18/11/2015
% Description:   Controller for Search, Approach and Learn Timing of the
%                CoS in a mobile robot doing a sequence between diferent 
%                positions, simulation in Matlab.
% Author:        Boris Duran
% Modifications: 
%}
function [hh, pp] = cosLearning_modified()
  pp = initParams();

  kk.aII = initKernels( pp.fSizeW, 20, pp.fI.kII );
  kk.aSS = initKernels( pp.fSizeW, 20, pp.fS.kSS );
  kk.aSI = initKernels( pp.fSizeW, 20, pp.fS.kSI );

  kk.lAA = initKernels( pp.fSize, 20, pp.fA.kAA );
  kk.lBA = initKernels( pp.fSize, 20, pp.fB.kBA );
  kk.lCC = initKernels( pp.fSize, 20, pp.fC.kCC );
  kk.lCB = initKernels( pp.fSize, 20, pp.fC.kCB );

  [hh, pp] = mainLoop( pp, kk );
%   showHistory( hh, pp );
end

function [hh, pp] = mainLoop( pp, kk )
  global rr ff nn;

  rr.Ts = pp.rr.Ts; % Target coordinates
  rr.tetha = 0;     % Target's initial conditions
  % ----------------------- Plotting stuff -------------------------
  %{.
  figure('Position',[1 50 400 300]);
  rFig.ax0 = axes('Position',[0.08 0.08 0.9 0.9]);
  fig.h = figure('Position',[1 520 800 450]);
  fig.ax1 = axes('Position',[0.05 0.08 0.20 0.85]);
  fig.ax2(1,1) = axes('Position',[0.52 0.79 0.24 0.16]);
  fig.ax2(1,2) = axes('Position',[0.30 0.59 0.24 0.16]);
  fig.ax2(1,3) = axes('Position',[0.34 0.34 0.24 0.16]);
  fig.ax2(1,4) = axes('Position',[0.68 0.34 0.24 0.16]);
  fig.ax2(1,5) = axes('Position',[0.74 0.59 0.24 0.16]);
  fig.ax3 = axes('Position',[0.50 0.05 0.34 0.2]);
  tCentral = ({'\bf \fontsize{14}Trial: 0'; 'Step: 0'});
  fig.ant = annotation( fig.h,'textbox',[0.57 0.58 0.12 0.1],'String', tCentral);
  %pause;
  %}
  % -------------------------- History ----------------------------
  %{.
  hh.int = zeros( pp.nTrials*pp.tMax, pp.nSize );
  hh.mem = zeros( pp.nTrials*pp.tMax, pp.nSize );
  hh.psp = zeros( pp.nTrials*pp.tMax, pp.nSize );
  hh.cos = zeros( pp.nTrials*pp.tMax, pp.nSize );
  hh.cod = zeros( pp.nTrials*pp.tMax, pp.nSize );
  hh.fA = zeros( pp.nTrials*pp.tMax, pp.fSize, pp.nSize );
  hh.fB = zeros( pp.nTrials*pp.tMax, pp.fSize, pp.nSize );
  hh.fC = zeros( pp.nTrials*pp.tMax, pp.fSize, pp.nSize );
  hh.fI = zeros( pp.nTrials*pp.tMax, pp.fSizeW );
  hh.fS = zeros( pp.nTrials*pp.tMax, pp.fSizeW );
  hh.rr = zeros( pp.nTrials*pp.tMax, 3 ); % [Rx Ry phi Tx Ty]
  %}
  % ---------------------------------------------------------------
  ff.lB = zeros( pp.nSize, pp.fSize );  % Setting memory trace to zero!
  for k = pp.firstTrial:pp.nTrials,
    % ------- Initialization for each trial ------
    rr.Rx = 0; rr.Ry = 0; rr.phi = 0;   % Robot's initial conditions
    rr.vel = 0;  rr.dT = 1000;          % Robot's initial conditions
    nn.int = pp.nInt.h * ones( 1, pp.nSize );
    nn.mem = pp.nMem.h * ones( 1, pp.nSize );
    nn.psp = pp.nPsp.h * ones( 1, pp.nSize );
    nn.cos = pp.nCoS.h * ones( 1, pp.nSize ); 
    nn.cod = pp.nCoD.h * ones( 1, pp.nSize );
    ff.lA = pp.fA.h * ones( pp.nSize, pp.fSize );
    ff.lC = pp.fC.h * ones( pp.nSize, pp.fSize );
    ff.fI = pp.fI.h * ones(1,pp.fSizeW);
    ff.fS = pp.fS.h * ones(1,pp.fSizeW);
%     pp.inCoS = 0;
    pp.goSignal = zeros( 1, pp.nSize ); 
%     pp.goSignal(1,pp.nOrder(1)) = 2.75;
    pp.t_iA = zeros( 1, pp.nSize ); % tCounter for impulse to field A
    for t = 1:pp.tMax,
%       fprintf('%2d: tPsp=%4.2f\n', k, pp.nPsp.tau);
      % -----------------save data -----------------
      hh.int(t+(k-1)*pp.tMax,:) = nn.int; 
      hh.mem(t+(k-1)*pp.tMax,:) = nn.mem; 
      hh.psp(t+(k-1)*pp.tMax,:) = nn.psp;
      hh.cos(t+(k-1)*pp.tMax,:) = nn.cos; 
      hh.cod(t+(k-1)*pp.tMax,:) = nn.cod;
      hh.fA(t+(k-1)*pp.tMax,:,:) = ff.lA'; 
      hh.fB(t+(k-1)*pp.tMax,:,:) = ff.lB'; 
      hh.fC(t+(k-1)*pp.tMax,:,:) = ff.lC';
      hh.fI(t+(k-1)*pp.tMax,:) = ff.fI; 
      hh.fS(t+(k-1)*pp.tMax,:) = ff.fS;
      hh.rr(t+(k-1)*pp.tMax,:) = [rr.Rx rr.Ry rr.phi];% rr.Tx rr.Ty];
      % --------- Cognition --------
      %fprintf('getForces: ');
      %tic;
      ft = getForces( pp.rr, k );
      %toc
      % ---------- Motion ----------
      %fprintf('getRobot: ');
      %tic;
      getRobot( pp.rr, ft );
      %toc
      % --------- Learning ---------
      % fprintf('doLearning: ');
      %tic;
      pp = doLearning( pp, kk, rr, t );
      % toc
      % === Plotting stuff ===
      % fprintf('drawStuff: ');
      %tic;
      showDynamics( nn, ff, pp, k, t, fig )
      showRobot( rr, pp.rr, k, rFig.ax0 );
      drawnow;
      %toc
      if rem(t,10) == 0, fprintf('.'); end
    end
    fprintf('%d\n', k);
  end
 % showHistory(hh, pp);
 
  %% Plots the robot's path in the sequence
%{.
figure('Position',[1 250 400 300]);
ax0 = axes('Position',[0.08 0.08 0.9 0.9]);
plot(ax0, pp.rr.Ts(1,:), pp.rr.Ts(2,:), 'og', 'LineWidth', 4); 
hold(ax0, 'on');  %axis equal;
axis(ax0, [-80 80 -2 120]);
plot(ax0, pp.rr.obsList(1,1:2), pp.rr.obsList(1,3:4), 'r');
tIni = 1+(3-1)*pp.tMax;
tEnd = 500+(3-1)*pp.tMax;
plot(ax0, hh.rr(tIni:tEnd,1),hh.rr(tIni:tEnd,2), ':b');
hold(ax0, 'on');
tIni = 1+(4-1)*pp.tMax;
tEnd = 500+(4-1)*pp.tMax;
plot(ax0, hh.rr(tIni:tEnd,1),hh.rr(tIni:tEnd,2), ':r');
%}
%% Plots the time evolution of nodes
%{.
iEB = 4;
figure('Position',[1 400 600 300]);
ax0 = axes('Position',[0.05 0.12 0.92 0.78]);
plot(ax0, hh.int(:,iEB), 'LineWidth', 3);
hold(ax0,'on'); grid(ax0,'on');
plot(ax0, hh.psp(:,iEB),'m', 'LineWidth', 3); 
plot(ax0, hh.cos(:,iEB),'g', 'LineWidth', 3);
%plot(ax0, hh.mem(:,iEB),'c'); 
plot(ax0, hh.cod(:,iEB),'r', 'LineWidth', 3);
axis(ax0, [0 2500 -8 8]);
title('Activity of all nodes in EB#4 through all trials');
xlabel('Trials');
set(ax0,'XTick', 0:500:2500, 'XTickLabel',{'','1','2','3','4','5'});
%}

%% Plots the time evolution of fields in a 3D mesh
%{.
iEB = 5;
tt = 1:pp.nTrials * pp.tMax;
figure('Position',[1 400 600 300]);
ax0 = axes('Position',[0.1 0.12 0.92 0.78]);
[X,Y] = meshgrid(tt,pp.fMinW:pp.fMaxW);
mesh(ax0, X,Y, hh.fI');
axis(ax0, [0 2500 -45 45]);
title(ax0, 'Activity of intention field through all trials');
xlabel(ax0, 'Trials'); ylabel(ax0, 'Orientation');
set(ax0, 'XTick', 0:500:2500, 'XTickLabel',{'','1','2','3','4','5'}, ...
        'YTick', -45:15:45, 'YTickLabel',-180:60:180);
%}

%% Plots the time evolution of fields I&S in a 2D image
%{.
tt = 1:pp.nTrials * pp.tMax;
figure('Position',[1 400 650 300]);
ax0 = axes('Position',[0.08 0.12 0.9 0.78]);
imagesc(tt, pp.fMinW:pp.fMaxW, hh.fS', [-8 10]);
title(ax0, 'Activity of intention field through all trials');
xlabel(ax0, 'Trials'); ylabel(ax0, 'Time');
set(ax0, 'XTick', 0:500:2500, 'XTickLabel',{'','1','2','3','4','5'}, ...
        'YTick', -45:15:45, 'YTickLabel',-180:60:180,'YDir','rev', ...
        'YDir','rev');
%}

%% Plots the time evolution of fields B in a 2D image
%{.
tt = 1:pp.nTrials * pp.tMax;
figure('Position',[1 400 650 300]);
ax0 = axes('Position',[0.08 0.12 0.9 0.78]);
imagesc(tt, pp.fMin:pp.fMax, hh.fB(:,:,5)', [0 0.35]);
title(ax0, 'Activity of the memory trace field for EB#5 through all trials');
xlabel(ax0, 'Trials'); ylabel(ax0, 'Time');
set(ax0, 'XTick', 0:500:2500, 'XTickLabel',{'','1','2','3','4','5'}, ...
        'YTick', [0 10 36 62 88], ...
        'YTickLabel',{'','0','125','250','375'}, ...
        'YDir','normal');
%}
 
end

function pp = initParams()
  pp.tMax = 500;
  pp.nTrials = 5;
  pp.kTrial = 0;
  pp.firstTrial = 1;

  % ================== Parameters of Mobile Robot ==================
  pp.rr.rayList = [pi/3 pi/6 0 -pi/6 -pi/3]; % Beacons (e.g. range finders)
  pp.rr.rayRange = 20;                       % Max distance of detection
  pp.rr.robWidth = 2;                        % Robot width
  pp.rr.phi = -pi:pi/180:pi;                 % Field
  pp.rr.Ts = [ 0 -60 -40 40 60; 100 80 30 30 80];  % Targets coordinates
  pp.rr.obsList = [30 -10 10 70; 20 60 40 40]; % Obstacles coordinates (lines).
  pp.rr.nObs = 1;                            % Number of working obstacles
  pp.rr.obsTrials = zeros(1,pp.nTrials);     % Obstacle flag for each trial
  pp.rr.obsTrials(1,4) = 1;                  % If 1 -> obstacle present
  pp.rr.tao = 20;
  pp.rr.lambdaT = 10;    
  pp.rr.betha1 = 40;  pp.rr.betha2 = 60;    %
  pp.rr.Cv_tar = 15;  pp.rr.Cv_obs = 1000;  %
  
  % ======================= Parameters for nodes =========================
  pp.nSize = 5; 
  % ---- Intention node ----
  pp.nInt.tau = 5;        pp.nInt.beta = 10;      pp.nInt.h = -3;
  pp.nInt.kIntInt = 6;    pp.nInt.kIntCoS = 3.1;  pp.nInt.kIntCoD = 5;
  pp.nInt.kIntMem = 1.25; pp.nInt.g_inh = 1.5;   
  % ---- Memory nodes -----
  pp.nMem.tau = 10;     pp.nMem.beta = 5;     pp.nMem.h = -4; 
  pp.nMem.kMemMem = 5;  pp.nMem.kMemInt = 5;  %pp.nMem.g_inh = 2;   
  % ---- Preshape node -----
  pp.nPsp.tau = 500*ones(1,pp.nSize);  % Different tau for each EB  
  pp.nPsp.beta = 5;     pp.nPsp.h = 0; 
  % ------ CoS node -------
  pp.nCoS.tau = 5;        pp.nCoS.beta = 10;     pp.nCoS.h = -5;
  pp.nCoS.kCoSCoS = 3.5;    pp.nCoS.kCoSCoD = 5;   pp.nCoS.kCoSPsp = 3.5;
  pp.nCoS.kCoSMem = 1.0;  pp.nCoS.g_inh = 1.4;  pp.nCos.knCoSfCoS = 3.75; %g_inh = 1.75; 
  % ------ CoD node -------
  pp.nCoD.tau = 10;     pp.nCoD.beta = 5;  pp.nCoD.h = -5;
  pp.nCoD.kCoDCoD = 3;  pp.nCoD.kCoDCoS = 3;  pp.nCoD.kCoDPsp = 4;
  pp.nCoD.kCoDMem = 1.0;  
  % ===================== Parameters for sequence ========================
  pp.nOrder = [5 2 4];
  pp.nInt.kIntMem_prev = 3.25;
  % ---- Matrix: memory into ordinal nodes -----
  pp.mxIntMem = zeros(pp.nSize);
  pp.mxIntMem(5,5) = -pp.nInt.kIntMem;
  pp.mxIntMem(2,2) = -pp.nInt.kIntMem;
  pp.mxIntMem(4,4) = -pp.nInt.kIntMem;
  pp.mxIntMem(5,2) = pp.nInt.kIntMem_prev;
  pp.mxIntMem(2,5) = pp.nInt.kIntMem_prev;
  pp.mxIntMem(4,2) = pp.nInt.kIntMem_prev;

  % ------ Matrix: memory into CoS nodes -------
  pp.mxCoSMem = zeros(pp.nSize);
  pp.mxCoSMem(5,5) = pp.nCoS.kCoSMem;
  pp.mxCoSMem(2,2) = pp.nCoS.kCoSMem;
  pp.mxCoSMem(4,4) = pp.nCoS.kCoSMem;
  
  % ================== Parameters for associated fields ==================
  pp.fSizeW = 91;     pp.fMinW = -45;       pp.fMaxW = 45;
  % ------------------------- Intention field --------------------------
  pp.fI.tau = 5;            pp.fI.h = -5;           pp.fI.beta = 4;
  pp.fI.kII.c_exc = 10;     pp.fI.kII.s_exc = 1.5;
  pp.fI.kII.c_inh = 2.5;    pp.fI.kII.s_inh = 5;    pp.fI.kII.g_inh = 0.25;
  % ---------------------------- CoS field -----------------------------
  pp.fS.tau = 5;           pp.fS.h = -5;           pp.fS.beta = 5;
  pp.fS.kSS.c_exc = 15;     pp.fS.kSS.c_inh = 0;    pp.fS.kSS.g_inh = 0.5;
  pp.fS.kSS.s_exc = 3;      pp.fS.kSS.s_inh = 1;
  pp.fS.kSI.c_exc = 10;     pp.fS.kSI.s_exc = 2;
  pp.fS.kSI.c_inh = 5;     pp.fS.kSI.s_inh = 4;    pp.fS.kSI.g_inh = 0.3;
  % ==================== Parameters for learning fields ==================
  pp.fSize = 101;  pp.fMin = 0;    pp.fMax = 100;
  pp.xx = 0:pp.fMax;
  % ------------------------- Traveling Peak --------------------------
  pp.fA.tau = 5;          pp.fA.h = -2;         pp.fA.beta = 5;
  pp.fA.kAA.c_exc = 15;   pp.fA.kAA.s_exc = 2;
  pp.fA.kAA.c_inh = 0;    pp.fA.kAA.s_inh = 1;  pp.fA.kAA.g_inh = 0.95;
  pp.fA.kfAnInt = 3;
  % ------------------------- Long Term Memory ------------------------
  pp.fB.tau_build = 100;          pp.fB.tau_decay = 1200;
  pp.fB.kBB.c_exc = 1.5;
  pp.fB.kBA.c_exc = 30;
  pp.fB.kBA.s_exc = 3;
  pp.fB.kBA.c_inh = 0;
  pp.fB.kBA.s_inh = 1;
  pp.fB.kBA.g_inh = 0;
  % ---------------------------- Time Peak ----------------------------
  pp.fC.tau = 5;          pp.fC.h = -4;         pp.fC.beta = 2.5;
  pp.fC.kCC.c_exc = 15;   pp.fC.kCC.s_exc = 6;
  pp.fC.kCC.c_inh = 0;    pp.fC.kCC.s_inh = 8;  pp.fC.kCC.g_inh = 0.9;
  pp.fC.kCB.c_exc = 1;   pp.fC.kCB.s_exc = 3;
  pp.fC.kCB.c_inh = 0;    pp.fC.kCB.s_inh = 1;  pp.fC.kCB.g_inh = 0.0;
  pp.fC.tOut = pp.fMax*ones(1,pp.nSize); % Position of peak in fC [0 100]
  pp.tBias = 10;   % Bias for the travelling peak to start moving.
  % ===================================================================
end

function kk = initKernels( size, factor, pp )
  kk.range = min(round(factor * (pp.s_exc + pp.s_inh)), floor((size-1)/2));
  kk.data = pp.c_exc * gauss1Dnorm(-kk.range:kk.range, 0, pp.s_exc) ...
            - pp.c_inh * gauss1Dnorm(-kk.range:kk.range, 0, pp.s_inh) - pp.g_inh;
  kk.indx = [size - kk.range + 1 : size, 1:size, 1:kk.range];
end

function ft = getForces( pp, nTrial )
  global rr;% ff;

  ft.fO = 0; ft.U = 0;
  fOi = zeros( size(pp.rayList,2), size(pp.phi,2));
  ft.dO = pp.rayRange*ones(1,size(pp.rayList,2));
  % ------------ calculation of obtacles forcelets ------------
  for i = 1:size(pp.rayList,2)
    bX = [rr.Rx rr.Rx+pp.rayRange*cos(rr.phi + pp.rayList(1,i))];
    bY = [rr.Ry rr.Ry+pp.rayRange*sin(rr.phi + pp.rayList(1,i))];
    dOaux = pp.rayRange*ones(1,pp.nObs); % Saves all crossings per beacon
    if pp.obsTrials(1,nTrial) == 1,
      for ix = 1:pp.nObs % Loop with all obstacles 
        [ipX, ipY] = intersections(bX, bY, pp.obsList(ix,1:2), pp.obsList(ix,3:4));
        if(~isempty(ipX))
          dOaux(1,ix) = sqrt((ipX-rr.Rx)^2 + (ipY-rr.Ry)^2);
        end
      end
    end
    ft.dO(1,i) = min(dOaux); % If several crossings, work with the shortest
    lambdaO = pp.betha1*exp(-ft.dO(1,i)/pp.betha2);
    sigmaO = atan(tan(pi/12) + pp.robWidth/(ft.dO(1,i)+pp.robWidth));
    ft.fO = ft.fO + lambdaO*(-pp.rayList(i))*exp(-pp.rayList(i)^2 / (2*sigmaO^2));
    psiObs = rr.phi + pp.rayList(i);   % mod(phiR + p.ray(i),2*pi);
    fOi(i,:) = lambdaO*(pp.phi - psiObs).* ...
        exp(-(pp.phi - psiObs).^2 / (2*sigmaO^2));
    ft.U = ft.U + sigmaO*sigmaO*lambdaO*...
        (exp(-pp.rayList(i).^2 / (2*sigmaO^2)) - exp(-0.5));
  end
%   ff.Ophi = sum(fOi,1);
  % ------------- calculation of target forcelet -------------
  rr.tetha = atan2(rr.Ts(2,:) - rr.Ry, rr.Ts(1,:) - rr.Rx);
  % --- Sinusoidal attractive forcelet ---
  ft.fT = -pp.lambdaT * sin(rr.phi - rr.tetha);
%   ff.Tphi = -pp.lambdaT * sin(pp.phi - rr.tetha);
  % --- Triangular attractive forcelet ---
  % fT = -p.lambdaT * sawtooth(phiR - angT + pi/2,0.5);
  % fTphi = -p.lambdaT * sawtooth(p.phi - angT + pi/2,0.5);
  % ---------------------------------------------------------------------
%   ff.robPhi = ff.Ophi + ff.Tphi;
%   ft = [fO fT U dO];
end

function getRobot( pp, ft )
  global rr nn;
  
  rr.dT = 100;
  phiDot = 0;
  if any(nn.int>0)
    ix = find(nn.int>0);
    rr.dT = sqrt((rr.Ts(1,ix)-rr.Rx)^2 + (rr.Ts(2,ix)-rr.Ry)^2);
    phiDot = sum(ft.fO + ft.fT(1,ix));
  end
  % ------------- calculation of robot velocity --------------
%     vel = sigmoid(dT,50,p.rWidth); % Velocity independent of obstacles
  dOmin = min(ft.dO);%(1,4));   % Down: velocity dependent on target+obstacles
  alpha = (1/pi)*atan(100*ft.U);%(1,3));
  
  Vobs = sigmoid( dOmin, 1, 2*pp.robWidth );
  Cobs = pp.Cv_obs*(0.5+alpha);   
  gObs = -Cobs.*(rr.vel - Vobs);
  
  Vtar = sigmoid(rr.dT,20,pp.robWidth);
  Ctar = pp.Cv_tar*(0.5-alpha);   
  gTar = -Ctar.*(rr.vel - Vtar);
  
  if all(nn.int < 0) || rr.dT < 2, %
    rr.vel = 0;
  else
    rr.vel = rr.vel + (1/pp.tao)*( gObs + gTar );
  end
  % ---------------------- update robot ---------------------- 
  rr.phi = rr.phi + (1/pp.tao)*phiDot;%(1,phiRdeg);
  rr.phi = mod( rr.phi, 2*pi );
  rr.Rx = rr.Rx + rr.vel * cos(rr.phi);
  rr.Ry = rr.Ry + rr.vel * sin(rr.phi);    

end

function pp = doLearning( pp, kk, rr, t )
  global ff nn;
  ix = 0; ixFlag = false;
  if any(nn.int>0), 
    ix = find(nn.int>0); ixFlag = true; 
  end
  aRange = -kk.lAA.range:kk.lAA.range;
  % -------------- set go signal ---------------
  if t>5,% && t<30, 
    pp.goSignal(1,pp.nOrder(1)) = 3.25; 
%   else
%     pp.goSignal(1,pp.nOrder(1)) = 0; 
  end
  % ==================== get outputs =======================
  % ---------------- for nodes ---------------
  onInt = sigmoid( nn.int, pp.nInt.beta, 0 );
  onMem = sigmoid( nn.mem, pp.nMem.beta, 0 );
  onPsp = sigmoid( nn.psp, pp.nPsp.beta, 0 );
  onCoS = sigmoid( nn.cos, pp.nCoS.beta, 0 );
  onCoD = sigmoid( nn.cod, pp.nCoD.beta, 0 );
  % ---------------- for fields --------------
  ofA = sigmoid( ff.lA, pp.fA.beta, 0 );
  ofC = sigmoid( ff.lC, pp.fC.beta, 0 );
  conv_AA = zeros( pp.nSize, pp.fSize );
  conv_CC = zeros( pp.nSize, pp.fSize );
  conv_CB = zeros( pp.nSize, pp.fSize );
  conv_BA = zeros( pp.nSize, pp.fSize );
  for i = 1:pp.nSize
    if i == ix
      kBias = 2.0 * ( 1 - onCoS(ix) );
      kAdata = pp.fA.kAA.c_exc*gauss1Dnorm(aRange,kBias,pp.fA.kAA.s_exc)...
               - pp.fA.kAA.g_inh;
      conv_AA(i,:) = conv(ofA(i,kk.lAA.indx), kAdata, 'valid');
    else
      conv_AA(i,:) = conv(ofA(i,kk.lAA.indx), kk.lAA.data, 'valid');
    end
    conv_CC(i,:) = conv(ofC(i,kk.lCC.indx), kk.lCC.data, 'valid');
    conv_CB(i,:) = conv(ff.lB(i, kk.lCB.indx ), kk.lCB.data, 'valid');
    conv_BA(i,:) = conv(ofA(i, kk.lBA.indx ), kk.lBA.data, 'valid');
  end
  
  ofI = sigmoid( ff.fI, pp.fI.beta, 0 );
  ofS = sigmoid( ff.fS, pp.fS.beta, 0 );
  conv_II = conv(ofI(:,kk.aII.indx), kk.aII.data, 'valid');
  conv_SI = conv(ofI(:,kk.aSI.indx), kk.aSI.data, 'valid');
  conv_SS = conv(ofS(:,kk.aSS.indx), kk.aSS.data, 'valid');
  % ==================== update dynamics =======================
  % ----------------------- for nodes ---------------------
  nn.int = nn.int + 1/pp.nInt.tau * ( -nn.int + pp.nInt.h ...
          + pp.nInt.kIntInt * onInt - pp.nInt.g_inh * sum(onInt) ... 
          + (pp.mxIntMem * onMem')' + pp.goSignal ...
          - pp.nInt.kIntCoS * onCoS - pp.nInt.kIntCoD * onCoD );
  if ixFlag, 
    nn.psp(1,ix) = nn.psp(1,ix) ...
                   + 1/pp.nPsp.tau(1,ix) * ( -nn.psp(1,ix) + onInt(1,ix) ); 
  end
  nn.mem = nn.mem + 1/pp.nMem.tau*( -nn.mem + pp.nMem.h ...
          + pp.nMem.kMemMem * onMem + pp.nMem.kMemInt * onInt ); % 
  inCoS = sigmoid( sum(ofS), 10, 0.5 );
  nn.cos = nn.cos + 1/pp.nCoS.tau*( -nn.cos + pp.nCoS.h ...
          + pp.nCoS.kCoSCoS * onCoS - pp.nCoS.kCoSCoD * onCoD ...
          + pp.nCoS.kCoSPsp * onPsp + pp.nCoS.kCoSMem * onMem ...
          + pp.nCos.knCoSfCoS * inCoS - pp.nCoS.g_inh * max(onCoS));
  nn.cod = nn.cod + 1/pp.nCoD.tau*( -nn.cod + pp.nCoD.h ...
          + pp.nCoD.kCoDCoD * onCoD - pp.nCoD.kCoDCoS * onCoS ...
          + pp.nCoD.kCoDPsp * onPsp + pp.nCoD.kCoDMem * onMem ); % 
  % ----------------------- for fields ---------------------
  % --- field A ---
  iA = zeros( pp.nSize, pp.fSize );
  if ixFlag,
    if pp.t_iA(1,ix) < 5,
      pp.t_iA(1,ix) = pp.t_iA(1,ix) + 1;
      iA(ix,:) = 4 * gauss1D( 1:pp.fSize, pp.tBias, 2 );
    end
  end
  ff.lA = ff.lA + 1/pp.fA.tau * ( -ff.lA + pp.fA.h + iA + conv_AA );
  
  % --- field B ---
  activeA = zeros( pp.nSize, pp.fSize );
  if ixFlag
    activeA(ix,:) = ff.lA(ix,:) > 0;
  end
  if any( sum(activeA,1) )
  %  ff.lB = ff.lB + 1/pp.fB.tau_build * (-ff.lB + pp.fB.kBB.c_exc*ofA) .* activeA ...
  ff.lB = ff.lB + 1/pp.fB.tau_build * (-ff.lB + conv_BA) .* activeA ...
                    + 1/pp.fB.tau_decay * (-ff.lB) .* (1-activeA);
  end
  % --- field C ---
  ff.lC = ff.lC + 1/pp.fC.tau * ( -ff.lC + pp.fC.h + conv_CC + conv_CB );
  
  % --- field I ---
  ifI = zeros(1,pp.fSizeW); tetha = 0;
  if ixFlag
    tetha = ( rr.tetha(1,ix) * 180 / pi )/4;  % field size is 90, not 360
    ifI = onInt(1,ix) * 7.5 * gauss1D( pp.fMinW:pp.fMaxW, tetha, 1.1 );
  end
  ff.fI = ff.fI + 1/pp.fI.tau*(-ff.fI + pp.fI.h + ifI + conv_II);
  
  % --- field S ---
  gain = 10./(1+exp(3*(rr.dT-2)));
%   gain = 10./(1+exp(5*(rr.dT-1)));
%   phi = ( rr.phi * 180 / pi )/4;  % field size is only 90, not 360
  if tetha > 0, phi = ( rr.phi * 180 / pi )/4;
  else phi = ( (rr.phi-2*pi) * 180 / pi )/4; end
  ifS = gain * gauss1D( pp.fMinW:pp.fMaxW, phi, 1.1 );
  ff.fS = ff.fS + 1/pp.fS.tau*(-ff.fS + pp.fS.h + ifS + conv_SS + conv_SI);
  % ================ update parameters (learning) ================
  if ixFlag,
    if any( sum(ff.lC(ix,:) > 0) ),
      tDot = -pp.fC.tOut(1,ix) * sum( sigmoid(ff.lC(ix,:), 1, 0) ) ...
             + sum( (0:pp.fMax) .* sigmoid(ff.lC(ix,:), 1, 0) );
      pp.fC.tOut(1,ix) = pp.fC.tOut(1,ix) + 0.05 * tDot;
      pp.nPsp.tau(1,ix) = 5*(pp.fC.tOut(1,ix) - pp.tBias);
    end
  end

end

function showDynamics( nn, ff, pp, kTrial, tStep, fig )
  % ---------------------- Plotting all nodes -------------------------
  plot( fig.ax1, [0 pp.nSize+1], [0 0], '-.k', 'Linewidth', 2 ); 
  hold( fig.ax1, 'on' ); grid( fig.ax1, 'on' );
  plot( fig.ax1, nn.int, 'ob', 'MarkerSize', 8, 'Linewidth', 2 );
  plot( fig.ax1, nn.mem, 'dc', 'MarkerSize', 8, 'Linewidth', 2 );
  plot( fig.ax1, nn.psp, '.m', 'MarkerSize', 24 );
  plot( fig.ax1, nn.cos, 'sg', 'MarkerSize', 10, 'Linewidth', 2 );
  plot( fig.ax1, nn.cod, 'xr', 'MarkerSize', 10, 'Linewidth', 2 );
  set( fig.ax1, 'xlim', [0 pp.nSize+1], 'ylim', [-8,8], 'ytick', -8:8);
%     xlabel(fig.ax1, ['t: ' num2str(t)]);  
  hold( fig.ax1, 'off');
  % ------------------ Plotting learning fields -----------------------
  for i = 1 : pp.nSize
    [mx,ix] = max(ff.lC(i,:));
    plot( fig.ax2(1,i), [0 pp.fSize-1], [0 0], '-.k' ); 
    hold( fig.ax2(1,i), 'on' );    grid( fig.ax2(1,i), 'on' );
    plot( fig.ax2(1,i), pp.fMin:pp.fMax, ff.lA(i,:), 'b', 'Linewidth', 3);
    plot( fig.ax2(1,i), pp.fMin:pp.fMax, ff.lB(i,:), 'r', 'Linewidth', 3);
    plot( fig.ax2(1,i), pp.fMin:pp.fMax, ff.lC(i,:), 'm', 'Linewidth', 3);
    if any(ff.lC(i,:)>0), 
      plot( fig.ax2(1,i), [ix-1 ix-1], [0 mx], '.-k' ); 
      text( ix, mx+1, num2str(pp.nPsp.tau(1,i)) );
    end
    tNode = sprintf('Fields for EB#%d (tao:%3.2f)', i, pp.nPsp.tau(1,i));
    title(fig.ax2(1,i), tNode);%['\bf Fields for EB#' num2str(i)]);
    set( fig.ax2(1,i), 'xlim', [0 pp.fSize-1], 'ylim', [-10 10], ...
          'XTick', [0 10 36 62 88], ...
          'XTickLabel',{'','0','125','250','375','450',''});
%       'XTick', [0 10 50 100], 'XTickLabel',{'','0','120','240'});
    hold( fig.ax2(1,i), 'off');
  end
  % ------------------ Tracking trial & timestep ---------------- -------
  tCentral = ({['\bf \fontsize{14}Trial: ' num2str(kTrial)]; ...
              ['Step: ', num2str(tStep)]});
  set(fig.ant, 'String', tCentral);
  % ------------------ Plotting associated fields ----------------------
  plot( fig.ax3, [pp.fMinW pp.fMaxW], [0 0], '-.k', 'Linewidth', 2 ); 
  hold( fig.ax3, 'on' );    grid( fig.ax3, 'on' );
  plot( fig.ax3, pp.fMinW:pp.fMaxW, ff.fI );
  plot( fig.ax3, pp.fMinW:pp.fMaxW, ff.fS, 'r' );
  title(fig.ax3, 'Robot orientation');
  set( fig.ax3, 'xlim', [pp.fMinW pp.fMaxW], 'ylim', [-10 10], ...
    'XTick', pp.fMinW:22.5:pp.fMaxW, 'XTickLabel', -180:90:180);  
%   xlabel(fig.ax3, 'Orientation' );
  hold( fig.ax3, 'off' );
end

function showRobot( rr, pp, kTrial, ax0 )
  Xc = rr.Rx;   Yc = rr.Ry;   fai = rr.phi;
  nb = size(pp.rayList,2); Beacon = zeros(nb,2);
  robot_width = pp.robWidth;
  beacon_range = pp.rayRange;

  % [Robot] = Robotplot(Xc, Yc, fai, robot_width);
  % xe, ye is the location of the robot and phe is the angle of x-axis
  % of the robot with respect to the base.
  Dia = robot_width*(2^0.5); 

  Robot(1,1) = Xc + Dia*cos(fai + pi/4);
  Robot(1,2) = Yc + Dia*sin(fai + pi/4);

  Robot(1,3) = Xc + Dia*cos(fai + 3*pi/4);
  Robot(1,4) = Yc + Dia*sin(fai + 3*pi/4);

  Robot(1,5) = Xc + Dia*cos(fai - 3*pi/4);
  Robot(1,6) = Yc + Dia*sin(fai - 3*pi/4);

  Robot(1,7) = Xc + Dia*cos(fai - pi/4);
  Robot(1,8) = Yc + Dia*sin(fai - pi/4);

  for i = 1:nb
      Beacon(i,1) = Xc + beacon_range*cos(fai + pp.rayList(1,i));
      Beacon(i,2) = Yc + beacon_range*sin(fai + pp.rayList(1,i));
  end

  %*** Create the 4 lines that draw the box of the mobile robot ***

  Robot1x = [Robot(1,1) Robot(1,3)];
  Robot1y = [Robot(1,2) Robot(1,4)];

  Robot2x = [Robot(1,3) Robot(1,5)];
  Robot2y = [Robot(1,4) Robot(1,6)];

  Robot3x = [Robot(1,5) Robot(1,7)];
  Robot3y = [Robot(1,6) Robot(1,8)];

  Robot4x = [Robot(1,7) Robot(1,1)];
  Robot4y = [Robot(1,8) Robot(1,2)];

  Beacon1 = [Xc Beacon(1,1); Yc Beacon(1,2)];
  Beacon2 = [Xc Beacon(2,1); Yc Beacon(2,2)];
  Beacon3 = [Xc Beacon(3,1); Yc Beacon(3,2)];
  Beacon4 = [Xc Beacon(4,1); Yc Beacon(4,2)];
  Beacon5 = [Xc Beacon(5,1); Yc Beacon(5,2)];

  plot(ax0, pp.Ts(1,:), pp.Ts(2,:), 'og', 'LineWidth', 4); 
  hold(ax0, 'on');  %axis equal;
  axis(ax0, [-80 80 -2 120]);
  if pp.obsTrials(1,kTrial) == 1,
    for j = 1:pp.nObs, 
      plot(ax0, pp.obsList(j,1:2),pp.obsList(j,3:4),'r');  
    end
  end
%   plot (Xc,Yc,'r-','LineWidth',1);
  plot(ax0, Robot1x, Robot1y,'b-','LineWidth',4);
  plot(ax0, Robot2x, Robot2y,'b-','LineWidth',1);
  plot(ax0, Robot3x, Robot3y,'b-','LineWidth',4);
  plot(ax0, Robot4x, Robot4y,'b-','LineWidth',1);
  plot(ax0, Beacon1(1,:), Beacon1(2,:),'m','LineWidth',1);
  plot(ax0, Beacon2(1,:), Beacon2(2,:),'m','LineWidth',1);
  plot(ax0, Beacon3(1,:), Beacon3(2,:),'m','LineWidth',1);
  plot(ax0, Beacon4(1,:), Beacon4(2,:),'m','LineWidth',1);
  plot(ax0, Beacon5(1,:), Beacon5(2,:),'m','LineWidth',1);
  hold(ax0, 'off');

end

function showHistory( hh, pp)
%% This block could be used from the console after running the main loop

  figure('Position',[1 1 400 300]);
  rFig.ax0 = axes('Position',[0.08 0.08 0.9 0.9]);
  fig.h = figure('Position',[1 520 800 450]);
  fig.ax1 = axes('Position',[0.05 0.08 0.20 0.85]);
  fig.ax2(1,1) = axes('Position',[0.52 0.79 0.24 0.16]);
  fig.ax2(1,2) = axes('Position',[0.30 0.59 0.24 0.16]);
  fig.ax2(1,3) = axes('Position',[0.34 0.34 0.24 0.16]);
  fig.ax2(1,4) = axes('Position',[0.68 0.34 0.24 0.16]);
  fig.ax2(1,5) = axes('Position',[0.74 0.59 0.24 0.16]);
  fig.ax3 = axes('Position',[0.50 0.05 0.34 0.2]);
  tCentral = ({'\bf \fontsize{14}Trial: 0'; 'Step: 0'});
  fig.ant = annotation( fig.h,'textbox',[0.57 0.58 0.12 0.1],'String', tCentral);
%%
  for k = 4:pp.nTrials,
    for t = 1:pp.tMax,
      % ---------------------- Plotting all nodes -------------------------
      plot( fig.ax1, [0 pp.nSize+1], [0 0], '-.k', 'Linewidth', 2 ); 
      hold( fig.ax1, 'on' ); grid( fig.ax1, 'on' );
      plot( fig.ax1, hh.int(t+(k-1)*pp.tMax,:), 'ob', 'MarkerSize', 8,'Linewidth', 2);
      plot( fig.ax1, hh.mem(t+(k-1)*pp.tMax,:), 'dc', 'MarkerSize', 8,'Linewidth', 2);
      plot( fig.ax1, hh.psp(t+(k-1)*pp.tMax,:), '.m', 'MarkerSize', 24 );
      plot( fig.ax1, hh.cos(t+(k-1)*pp.tMax,:), 'sg', 'MarkerSize', 10,'Linewidth', 2);
      plot( fig.ax1, hh.cod(t+(k-1)*pp.tMax,:), 'xr', 'MarkerSize', 10,'Linewidth', 2);
      set( fig.ax1, 'xlim', [0 pp.nSize+1], 'ylim', [-8,8],'ytick',-8:8);
      title(fig.ax1, 'Nodes');  hold( fig.ax1, 'off');
      % ------------------ Plotting learning fields -----------------------
      for i = 1 : pp.nSize
        [mx, ix] = max(hh.fC(t+(k-1)*pp.tMax,:,i));
        plot( fig.ax2(1,i), [0 pp.fSize-1], [0 0], '-.k', 'Linewidth', 2 ); 
        hold( fig.ax2(1,i), 'on' );    grid( fig.ax2(1,i), 'on' );
        plot( fig.ax2(1,i), pp.fMin:pp.fMax, hh.fA(t+(k-1)*pp.tMax,:,i) );
        plot( fig.ax2(1,i), pp.fMin:pp.fMax, 10*hh.fB(t+(k-1)*pp.tMax,:,i),'r');
        plot( fig.ax2(1,i), pp.fMin:pp.fMax, hh.fC(t+(k-1)*pp.tMax,:,i),'m');
        if any(hh.fC(t+(k-1)*pp.tMax,:,i)>0), 
          plot( fig.ax2(1,i), [ix-1 ix-1], [0 mx], '.-k' ); 
          text( ix, mx+1, num2str(pp.nPsp.tau(1,i)) );
        end
        tNode = sprintf('Node #%d (tao:%3.2f)', i, pp.nPsp.tau(1,i));
        title(fig.ax2(1,i), tNode);%['\bf Node #' num2str(i)]);
        set( fig.ax2(1,i), 'xlim', [0 pp.fSize-1], 'ylim', [-10 10], ...
          'XTick', [0 10 36 62 88], ...
          'XTickLabel',{'','0','125','250','375','450',''});
        hold( fig.ax2(1,i), 'off');
      end
      % ------------------ Tracking trial & timestep ---------------- -----
      tCentral = ({['\bf \fontsize{14}Trial: ' num2str(k)]; ...
                  ['Step: ', num2str(t)]});
      set(fig.ant, 'String', tCentral);
      % ------------------ Plotting associated fields ---------------------
      plot( fig.ax3, [pp.fMinW pp.fMaxW], [0 0], '-.k', 'Linewidth', 2 ); 
      hold( fig.ax3, 'on' );    grid( fig.ax3, 'on' );
      plot( fig.ax3, pp.fMinW:pp.fMaxW, hh.fI(t+(k-1)*pp.tMax,:) );
      plot( fig.ax3, pp.fMinW:pp.fMaxW, hh.fS(t+(k-1)*pp.tMax,:), 'r' );
      title(fig.ax3, '\bf Robot orientation');
      set( fig.ax3, 'xlim', [pp.fMinW pp.fMaxW], 'ylim', [-10 10], ...
          'XTick', pp.fMinW:22.5:pp.fMaxW, 'XTickLabel', -180:90:180);
      hold( fig.ax3, 'off');
      % -------------------------------------------------------------------
%       rr.Rx = hh.rr(t+(k-1)*pp.tMax,1); 
%       rr.Ry = hh.rr(t+(k-1)*pp.tMax,2); 
%       rr.phi = hh.rr(t+(k-1)*pp.tMax,3); 
%       showRobot( rr, pp.rr, k, rFig.ax0 );
      drawnow();
    end
  end

end

%% ======= Supporting functions ========
function g = gauss1Dnorm(range_x, mu, sigma)
  g = exp(-0.5 * (range_x-mu).^2 / sigma^2);
  if any(g)
    g = g / sum(g);
  end
end

function g = gauss1D(range_x, mu, sigma)
  g = exp(-0.5 * (range_x-mu).^2 / sigma^2);
end

function s = sigmoid(x,beta,x0)
  s = 1./ (1 + exp(-beta*(x-x0)));
  
end