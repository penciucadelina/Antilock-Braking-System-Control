%% Model Predictive Control pentru sistem ABS
% Preluare date 
t = double(DateIdentificare(:,1));          % timp
y = double(DateIdentificare(:,2));          % semnalul de intrare
u = double(t > 0);                          % semnal de iesire
Te = t(10) - t(9);                          % perioada de esantionare

% afisare grafic
plot(t, u, 'LineWidth', 2)                 
hold on
plot(t, y, 'LineWidth', 2)                  
grid on
title('Datele achizitionate')
xlabel('Timp [s]')                         
ylabel('Semnale')                       
legend('Intrare (treapta)', 'Iesire (slip)')
%% Identificare sistem folosind OE si optimizare cu PEM

% pregatire date
date_id = iddata(y, u, Te);  
% Model OE: ordin A=2, B=2, timp mort=0
model_initial = oe(date_id, [2 2 0]);       
model_optim = pem(date_id, model_initial);  

%% Comparatie intre iesirea modelului si datele reale folosind compare
figure;
compare(model_optim, date_id);         
grid on;
title('Comparare model identificat vs. date reale');

%% Simulare determinista a modelului identificat (fara zgomot)
[y_sim, t_sim] = sim(model_optim, date_id);  

figure;
plot(t, y, 'b', 'LineWidth', 1.5); hold on;
plot(t, y_sim.y, 'r--', 'LineWidth', 1.5); grid on;

legend('Slip real', 'Slip model');
xlabel('Timp [s]');
ylabel('Slip');
title('Comparatie intre date reale si modelul identificat');    

%% Convertire la spatiul starilor
sys = ss(model_optim);          

% Matricile modelului in spatiul starilor
A = sys.A;
B = sys.B;
C = sys.C;
D = sys.D;
%% Functia de transfer obtinuta
H = tf(model_optim)
%% Configurare si simulare MPC 

% Pozitia unde incepe aplicarea treptei
startTreapta = find(u == 1, 1);

% Eliminare transfer directa intre intrare si iesire pt a putea folosi MPC
sys.D = 0;                  
sys = ss(sys.A, sys.B, sys.C, sys.D, Te);  

% Creare obiect MPC
mpc_ctrl = mpc(sys, Te);

% Configurare orizonturi de predictie si control
mpc_ctrl.PredictionHorizon = 30;     % (~0.27s)
mpc_ctrl.ControlHorizon = 3;         % control efectiv pe 3 pasi

% Setare ponderi in functia de cost
mpc_ctrl.Weights.ManipulatedVariables = 0;          % nu penalizeaza valoarea comenzii
mpc_ctrl.Weights.ManipulatedVariablesRate = 0.3;    % penalizeaza variatii bruste
mpc_ctrl.Weights.OutputVariables = 2;               % penalizeaza abateri fata de referinta

% Impunere limite fizice pe comanda
mpc_ctrl.MV.Min = 0;
mpc_ctrl.MV.Max = 1;


%% Simulare 
% initializare variabile pentru simulare in bucla inchisa

N = length(t);               % nr total de esantioane
y_mpc = zeros(N,1);          % iesirea controlata
u_mpc = zeros(N,1);          % comanda aplicata
r = zeros(N,1);              % referinta pentru iesire (slip)
r(startTreapta:end) = 0.8;   % referinta aplicata dupa treapta

stare = mpcstate(mpc_ctrl);   % stare interna a regulatorului MPC

% simulare bucla inchisa MPC

for k = 1:N
    % iesire masurata la pasul curent
    yk = sys.C * stare.Plant;
    
    % aplicare control dupa momentul treptei
    if k >= startTreapta
        uk = mpcmove(mpc_ctrl, stare, yk, r(k));  % calcul comanda
        uk = min(max(uk, mpc_ctrl.MV.Min), mpc_ctrl.MV.Max);  % saturare
    else
        uk = 0;
    end

    % actualizare stare si iesire
    stare.Plant = sys.A * stare.Plant + sys.B * uk;
    y_mpc(k) = sys.C * stare.Plant;
    u_mpc(k) = uk;
end

% afisare rezultate

figure;
plot(t, u, 'LineWidth',2); hold on;
plot(t, y, 'LineWidth',2);
plot(t, y_mpc, 'LineWidth',2);
legend('Comanda (treapta)', 'Slip real', 'Slip controlat cu MPC');
xlabel('Timp [s]');
ylabel('Slip');
title('Controlul ABS cu MPC');
grid on;

figure;
plot(t, u_mpc, 'm', 'LineWidth', 1.5);
xlabel('Timp [s]');
ylabel('Comanda MPC (franare)');
title('Comanda aplicata de MPC');
grid on;


%% 
% Raspunsul sistemului la treapta 
step(H);
title('Raspunsul la treapta al sistemului identificat (in discret)');
xlabel('Timp [s]');
ylabel('Amplitudine');
grid on;
figure
step(H);
title('Raspunsul la treapta al sistemului identificat (in discret)');
xlabel('Timp [s]');
ylabel('Amplitudine');
grid on;

%% Am calculat si urmatoarele erori: 
% ISE (Integral Squared Error)– reflecta energia erorii si penalizeaza mai puternic abaterile mari;
% IAE (Integral Absolute Error)-ofera o imagine generala asupra abaterii totale fata de referinta;
% MAE (Maximum Absolute Error)– indica cea mai mare abatere instantanee de la referinta.
% Valorile obtinute arata ca sistemul controleaza eficient alunecarea

%% Evaluarea performantelor MPC

% Eroare de urmarire
eroare = r - y_mpc;

% calculul unor indici de performanta
ISE = sum(eroare.^2);           % Integral Squared Error
IAE = sum(abs(eroare));         % Integral Absolute Error
MAE = max(abs(eroare));         % Maximum Absolute Error

% Afisare in Command Window
fprintf('Evaluarea performantelor MPC:\n');
fprintf('ISE (Integral Squared Error): %.4f\n', ISE);
fprintf('IAE (Integral Absolute Error): %.4f\n', IAE);
fprintf('MAE (Maximum Absolute Error): %.4f\n', MAE);

% Graficul erorii
figure;
plot(t, eroare, 'r', 'LineWidth', 1.5);
xlabel('Timp [s]');
ylabel('Eroare (r - y_{mpc})');
title('Eroarea de urmarire a MPC');
grid on;


