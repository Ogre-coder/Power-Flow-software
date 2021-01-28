clear

% Setting Bases for the System
Sb = 100e6;

Vb1 = 13.47e3;
Vb2 = Vb1;
Vb3 = Vb2 * 600 / 15.58e3;
Vb4 = Vb3 * 208 / 480;
Vb5 = Vb2 * 480 / 12.48e3;
Vb6 = Vb5;

Zb1 = Vb1^2/Sb;
Zb2 = Zb1;
Zb3 = Vb3^2/Sb;
Zb4 = Vb4^2/Sb;
Zb5 = Vb5^2/Sb;
Zb6 = Vb6^2/Sb;

% Line Impedances
% Line 12
Rln12 = 0.53;
Lln12 = 1e-3 * 5.3;
Xln12 = 2*pi*60*Lln12;
Zln12 = Rln12 + Xln12*1i;
Zpu12 = Zln12 / Zb1;

% Line 23
Rln23 = 0.1;
Lln23 = 1e-3 * 0.53;
Xln23 = 2*pi*60*Lln23;
Zln23 = Rln23 + Xln23*1i;
Zpu23 = Zln23 / Zb2;

% Line 25
Rln25 = 0.105;
Lln25 = 1e-3 * 1.2;
Xln25 = 2*pi*60*Lln25;
Zln25 = Rln25 + Xln25*1i;
Zpu25 = Zln25 / Zb2;

% Line 56
Rln56 = 0.045;
Lln56 = 1e-3 * 0.5;
Xln56 = 2*pi*60*Lln56;
Zln56 = Rln56 + Xln56*1i;
Zpu56 = Zln56 / Zb5;

% Transformer Impedances
% Transformer 1
Zt1old = 15.58e3^2 / 25e6;
Zt1new = 13.47e3^2 / 100e6;
XRt1 = 7;
theta1 = atan(XRt1);
Zt1 = 0.00324 * (cos(theta1) + sin(theta1)*1i);
Zput1 = Zt1 * Zt1old / Zt1new

% Transformer 2
Zt2old = 12.48e3^2 / 15e6;
Zt2new = 13.47e3^2 / 100e6;
XRt2 = 7;
theta2 = atan(XRt2);
Zt2 = 0.0047 * (cos(theta2) + sin(theta2)*1i);
Zput2 = Zt2 * Zt2old / Zt2new

% Transformer 3
Zt3old = 480^2 / 2e6;
Zt3new = (13.47e3*600/15.58e3)^2 / 100e6;
XRt3 = 15;
theta3 = atan(XRt3);
Zt3= 0.05 * (cos(theta3) + sin(theta3)*1i);
Zput3 = Zt3 * Zt3old / Zt3new

% Total Path Impedances
Z12 = Zpu12;
Z23 = Zpu23 + Zput1;
Z25 = Zpu25 + Zput2;
Z34 = Zput3;
Z56 = Zpu56;

% Line Admittances
Y12 = Z12^-1;
Y23 = Z23^-1;
Y25 = Z25^-1;
Y34 = Z34^-1;
Y56 = Z56^-1;

% Admittance Matrix
Ybus = [ Y12,    -Y12,        0,      0,     0,      0;
        -Y12, Y12+Y23+Y25,  -Y23,     0,   -Y25,     0;
          0,     -Y23,     Y23+Y34, -Y34,    0,      0;
          0,       0,       -Y34,    Y34,    0,      0;
          0,     -Y25,        0,      0,  Y25+Y56, -Y56;
          0,       0,         0,      0,   -Y56,    Y56];

% Bus typing: type 1 = Slack bus, type 2 = Generator bus, type 3 = Load bus, type 4 = Constant Vmag
% Enter the data for the bus system you are using
% Each row is as follows: 
%           No.,  Type, Voltage, Angle, PGen,  QGen, PLoad,   QLoad,     Qmin,  Qmax
busdata = [ 1     1     1.0      0      0      0     0        0          0      0;
            2     3     1.0      0      0      0     0.0375   0.028125   0      0;
            3     2     1.0      0      0.025  0     0.0155   0.006     -0.008  0.008;
            4     3     1.0      0      0.0    0     0.008    0.005      0      0;
            5     3     1.0      0      0.0    0     0.00905  0.0022     0      0;
            6     2     1.0      0      0.005  0     0        0          0      0];

% Separating data into variables
bus = busdata(:,1)';
type = busdata(:,2)';
V = busdata(:,3)';
th = busdata(:,4)';
GenMW = busdata(:,5)';
GenMVAR = busdata(:,6)';
LoadMW = busdata(:,7)';
LoadMVAR = busdata(:,8)';
Qmin = busdata(:,9)';
Qmax = busdata(:,10)';

% Number of busses
nbus = max(bus);
% Total bus active power
P = GenMW - LoadMW;
% Total bus reactive power
Q = GenMVAR - LoadMVAR;

% Setting up variables for looping
Vprev = V;
error = 1;
m = 1;  % iteration variable

% while the error of the iteration is greater than 'x' (ex. 0.0001)
while error > 0.0001
    % Excluding the slack bus (bus 1)
    for i = 2:nbus
        sumYV = 0;
        % Calculating the sum of (Yik * Vk) for i ≠ k
        for k = 1:nbus
            if i ~= k
                sumYV = sumYV + Ybus(i,k)* V(k);
            end
        end
        % Calculating the reactive power for bus i
        if type(i) == 2
            Q(i) = -imag(conj(V(i))*(sumYV + Ybus(i,i)*V(i)));
            % Maintaining Qmax and Qmin
            if (Q(i) > Qmax(i)) || (Q(i) < Qmin(i))
                if Q(i) < Qmin(i)
                    Q(i) = Qmin(i);
                else
                    Q(i) = Qmax(i);
                end
                type(i) = 3;
            end
        end
        % Calculating the bus voltage
        V(i) = (1/Ybus(i,i))*((P(i)-1i*Q(i))/conj(V(i)) - sumYV);
        % Maintain voltage magnitude but change angle
        if type(i) == 2
            V(i) = pol2cart(abs(Vprev(i)), angle(V(i)));
        end
    end
    % Calculate error
    error = max(abs(abs(V) - abs(Vprev)));
    Vprev = V;
    % Storing values for later manipulation
    V1(m) = V(1);
    V2(m) = V(2);
    V3(m) = V(3);
    V4(m) = V(4);
    V5(m) = V(5);
    V6(m) = V(6);
    m = m + 1;
    
    % Stop if solution is not converging
    if m > 1000
        disp('Solution did not converge')
        break;
    end
end

% Outputting Values
m;
Voltages = [V1' V2' V3' V4' V5' V6'];
Vmag = abs(Voltages);
Vfinal = Vmag(end,:);
Ang = 180/pi*angle(Voltages);

% Creating variables for the data table
m = 1:10;
magV2 = round(abs(V2(1:10)),3);
angleV2 = angle(V2(1:10))*180/pi;
magV3 = abs(V3(1:10));
angleV3 = angle(V3(1:10))*180/pi;
magV4 = abs(V4(1:10));
angleV4 = angle(V4(1:10))*180/pi;
magV5 = abs(V5(1:10));
angleV5 = angle(V5(1:10))*180/pi;
magV6 = abs(V6(1:10));
angleV6 = angle(V6(1:10))*180/pi;

ColumnNames = {'V2','V2Angle','V3','V3Angle','V4','V4Angle','V5','V5Angle','V6','V6Angle'};
table(magV2',angleV2',magV3',angleV3',magV4',angleV4',magV5',angleV5',magV6',angleV6','VariableNames',ColumnNames);