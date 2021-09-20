clc
clear
close all

% The code is to simulate the dynamic of pedestrain using social force model.
% https://www.foxnews.com/us/new-york-apple-store-gunfire-triggers-panic-video-shows
% case reference: Apple Store case on 12 November 2016
% Author: Qiujia Liu
% Last edit on 6/24/2021.

%% SIMULATION PARAMETERS
% EVACUATION ENVIRONMENT
Lx=23;Ly=12.7;
Num_safe=2;% including 2 exits
gap_safe=2;
d_delta = 0.5;
SE=zeros(Num_safe,2);
SE(2,:)=[Lx,d_delta+gap_safe/2];
SE(1,:)=[Lx,Ly-d_delta-gap_safe/2];
Lx_o1=3;Ly_o1=1.2;
Num_o1=14;% inclusing 14 R obstacle 
Loc_o1=zeros(Num_o1,2);
for j=1:4
    for i=1:3
        Loc_o1(j*3+i-3,:)=[-1.5+i*(Lx_o1+1.7)+1.7,0.1+j*(Ly_o1+1.3)];
    end
end
Loc_o1(13,:)=[-1.5+4*(Lx_o1+1.7)+1.7,0.1+1*(Ly_o1+1.3)];
Loc_o1(14,:)=[-1.5+4*(Lx_o1+1.7)+1.7,0.1+4*(Ly_o1+1.3)];
x_u1=zeros(Num_o1,4);
y_u1=zeros(Num_o1,4);
for n =1:Num_o1
    x_u1(n,1)=Loc_o1(n,1)-Lx_o1/2;
    x_u1(n,2)=Loc_o1(n,1)+Lx_o1/2;
    x_u1(n,3)=x_u1(n,2);x_u1(n,4)=x_u1(n,1);
    y_u1(n,1)=Loc_o1(n,2)+Ly_o1/2;
    y_u1(n,4)=Loc_o1(n,2)-Ly_o1/2;
    y_u1(n,2)=y_u1(n,1);y_u1(n,3)=y_u1(n,4);
end
ro = 0.2;
Num_o2=24;% inclusing 24 F obstacle 
Loc_o2=zeros(Num_o2,2);
for j=1:3
    for i=1:8
        Loc_o2(j*8+i-8,:)=[15.4+i*2*ro+1.7,3.05+j*1.65];
    end
end
x_u2=zeros(Num_o2,4);
y_u2=zeros(Num_o2,4);
for n =1:Num_o2
    x_u2(n,1)=Loc_o2(n,1)-ro;
    x_u2(n,2)=Loc_o2(n,1)+ro;
    x_u2(n,3)=x_u2(n,2);x_u2(n,4)=x_u2(n,1);
    y_u2(n,1)=Loc_o2(n,2)+ro;
    y_u2(n,4)=Loc_o2(n,2)-ro;
    y_u2(n,2)=y_u2(n,1);y_u2(n,3)=y_u2(n,4);
end
%EVACUATION PEDESTRIAN
Num_p=54;
ri = 0.3.*ones(Num_p,1);
mi=80;
v_max=5; 
load('initial_pedes_co.mat','initial_pedes_co')
exit = [1	1   1 % Target exit: [ped_id,ped_index,exit:1-R/2-F]
2	2   1 
7	3   1 
9	4   1 
10	5   1 
12	6   1 
15	7   1 
16	8   1 
17	9   1 
21	10  1 
23	11  1 
25	12  1 
26	13  1 
31	14  1 
32	15  1 
33	16  1 
34	17  1 
40	18  1 
65	19  1 
68	20  1 
3	21  2 
4	22  2 
5	23  2 
6	24  2 
8	25  1 
11	26  1 
13	27  1 
14	28  1 
18	29  1 
19	30  1 
20	31  1 
22	32  1 
24	33  1 
27	34  1 
28	35  1 
29	36  1 
30	37  1 
35	38  1 
36	39  1 
37	40  1 
38	41  1 
39	42  1 
41	43  1 
42	44  1 
43	45  1 
44	46  1 
47	47  1 
48	48  1 
49	49  1 
50	50  1 
66	51  1 
67	52  1 
69	53  1 
70	54  1];

%% PARAMETERS IN GOVERNING EQUATION 
t_i=0.5;
Ai=2000;
Bi=0.08;
k=1.2E5;
kappa=2.4E5;

%% INITIALIZATION
Vel=zeros(Num_p,2);
Mar=zeros(Num_p,1);
Loc=initial_pedes_co(:,2:3);
Record=[zeros(Num_p,1),(1:Num_p)',zeros(Num_p,1),Loc]; % Time,id,v,x,y

%% SIMULATION STARTS
TimeStep=200000;
dt=0.001;
for steps=1:TimeStep
    % calculate the force applied on each person
    F2=zeros(Num_p,2);
    Vel_2=zeros(Num_p,2);
    F3=zeros(Num_p,2);
    F5=zeros(Num_p,2); 
    % calculate F2 to desired destination
    for i=1:Num_p
        if Mar(i)==1
            continue;
        end
        if Loc(i,2)>SE(exit(i,3),2)-gap_safe/2+ri(i) && Loc(i,2)<SE(exit(i,3),2)+gap_safe/2-ri(i)
            d_2=Lx-Loc(i,1);
            Vel_2(i,:)=[1,0];
        else
            d_2=sqrt(sum((SE(exit(i,3),:)-Loc(i,:)).^2));
            Vel_2(i,:)=(SE(exit(i,3),:)-Loc(i,:))/d_2;
        end
        F2(i,:)=mi*(v_max*Vel_2(i,:)-Vel(i,:))/t_i;
    end
    % calculate F3 & F5
    for i=1:Num_p
        if Mar(i)==1
            continue;
        end
        % calculate F3 to obstacle R
        % upper wall
        niw=[0,-1];
        tiw=[1,0];
        for n=1:Num_o1
            if Loc(i,1)<=Loc_o1(n,1)+Lx_o1/2+ri(i) && Loc(i,1)>=Loc_o1(n,1)-Lx_o1/2-ri(i) && Loc(i,2)<=Loc_o1(n,2)-Ly_o1/2
                diw=Loc_o1(n,2)-Ly_o1/2-Loc(i,2);
                if diw>ri(i)
                    F3(i,:)=F3(i,:)+Ai*exp((ri(i)-diw)/Bi)*niw;
                else
                    F3(i,:)=F3(i,:)+k*(ri(i)-diw)*niw+Ai*exp((ri(i)-diw)/Bi)*niw-kappa*(ri(i)-diw)*sum((Vel(i,:).*tiw))*tiw;
                end
            end
        end
        for n=1:Num_o2
            if Loc(i,1)<=Loc_o2(n,1)+ro+ri(i) && Loc(i,1)>=Loc_o2(n,1)-ro-ri(i) && Loc(i,2)<=Loc_o2(n,2)-ro
                diw=Loc_o2(n,2)-ro-Loc(i,2);
                if diw>ri(i)
                    F3(i,:)=F3(i,:)+Ai*exp((ri(i)-diw)/Bi)*niw;
                else
                    F3(i,:)=F3(i,:)+k*(ri(i)-diw)*niw+Ai*exp((ri(i)-diw)/Bi)*niw-kappa*(ri(i)-diw)*sum((Vel(i,:).*tiw))*tiw;
                end
            end
        end
        diw=Ly-Loc(i,2);
        if diw>ri(i)
            F3(i,:)=F3(i,:)+Ai*exp((ri(i)-diw)/Bi)*niw;
        else
            F3(i,:)=F3(i,:)+k*(ri(i)-diw)*niw+Ai*exp((ri(i)-diw)/Bi)*niw-kappa*(ri(i)-diw)*sum((Vel(i,:).*tiw))*tiw;
        end
        % lower wall
        niw=[0,1];
        tiw=[-1,0];
        for n=1:Num_o1
            if Loc(i,1)<=Loc_o1(n,1)+Lx_o1/2+ri(i) && Loc(i,1)>=Loc_o1(n,1)-Lx_o1/2-ri(i) && Loc(i,2)>=Loc_o1(n,2)+Ly_o1/2
                diw=Loc(i,2)-(Loc_o1(n,2)+Ly_o1/2);
                if diw>ri(i)
                    F3(i,:)=F3(i,:)+Ai*exp((ri(i)-diw)/Bi)*niw;
                else
                    F3(i,:)=F3(i,:)+k*(ri(i)-diw)*niw+Ai*exp((ri(i)-diw)/Bi)*niw-kappa*(ri(i)-diw)*sum((Vel(i,:).*tiw))*tiw;
                end
            end
        end
        for n=1:Num_o2
            if Loc(i,1)<=Loc_o2(n,1)+ro+ri(i) && Loc(i,1)>=Loc_o2(n,1)-ro-ri(i) && Loc(i,2)>=Loc_o2(n,2)+ro
                diw=Loc(i,2)-(Loc_o2(n,2)+ro);
                if diw>ri(i)
                    F3(i,:)=F3(i,:)+Ai*exp((ri(i)-diw)/Bi)*niw;
                else
                    F3(i,:)=F3(i,:)+k*(ri(i)-diw)*niw+Ai*exp((ri(i)-diw)/Bi)*niw-kappa*(ri(i)-diw)*sum((Vel(i,:).*tiw))*tiw;
                end
            end
        end
        diw=Loc(i,2);
        if diw>ri(i)
            F3(i,:)=F3(i,:)+Ai*exp((ri(i)-diw)/Bi)*niw;
        else
            F3(i,:)=F3(i,:)+k*(ri(i)-diw)*niw+Ai*exp((ri(i)-diw)/Bi)*niw-kappa*(ri(i)-diw)*sum((Vel(i,:).*tiw))*tiw;
        end 
        % left wall
        niw=[1,0];
        tiw=[0,1];
        for n=1:Num_o1
            if Loc(i,1)>=Loc_o1(n,1)+Lx_o1/2 && Loc(i,2)>=Loc_o1(n,2)-Ly_o1/2-ri(i) && Loc(i,2)<=Loc_o1(n,2)+Ly_o1/2+ri(i)
                diw=Loc(i,1)-(Loc_o1(n,1)+Lx_o1/2);
                if diw>ri(i)
                    F3(i,:)=F3(i,:)+Ai*exp((ri(i)-diw)/Bi)*niw;
                else
                    F3(i,:)=F3(i,:)+k*(ri(i)-diw)*niw+Ai*exp((ri(i)-diw)/Bi)*niw-kappa*(ri(i)-diw)*sum((Vel(i,:).*tiw))*tiw;
                end
            end
        end
        for n=1:Num_o2
            if Loc(i,2)<=Loc_o2(n,2)+ro+ri(i) && Loc(i,2)>=Loc_o2(n,2)-ro-ri(i) && Loc(i,1)>=Loc_o2(n,1)+ro
                diw=Loc(i,1)-(Loc_o2(n,1)+ro);
                if diw>ri(i)
                    F3(i,:)=F3(i,:)+Ai*exp((ri(i)-diw)/Bi)*niw;
                else
                    F3(i,:)=F3(i,:)+k*(ri(i)-diw)*niw+Ai*exp((ri(i)-diw)/Bi)*niw-kappa*(ri(i)-diw)*sum((Vel(i,:).*tiw))*tiw;
                end
            end
        end
        diw=Loc(i,1);
        if diw>ri(i)
            F3(i,:)=F3(i,:)+Ai*exp((ri(i)-diw)/Bi)*niw;
        else
            F3(i,:)=F3(i,:)+k*(ri(i)-diw)*niw+Ai*exp((ri(i)-diw)/Bi)*niw-kappa*(ri(i)-diw)*sum((Vel(i,:).*tiw))*tiw;
        end
        % right wall (in the gap area, there is not interaction)
        niw=[-1,0];
        tiw=[0,-1];
        for n=1:Num_o1
            if Loc(i,1)<=Loc_o1(n,1)-Lx_o1/2 && Loc(i,2)>=Loc_o1(n,2)-Ly_o1/2-ri(i) && Loc(i,2)<=Loc_o1(n,2)+Ly_o1/2+ri(i)
                diw=Loc_o1(n,1)-Lx_o1/2-Loc(i,1);
                if diw>ri(i)
                    F3(i,:)=F3(i,:)+Ai*exp((ri(i)-diw)/Bi)*niw;
                else
                    F3(i,:)=F3(i,:)+k*(ri(i)-diw)*niw+Ai*exp((ri(i)-diw)/Bi)*niw-kappa*(ri(i)-diw)*sum((Vel(i,:).*tiw))*tiw;
                end
            end
        end
        for n=1:Num_o2
            if Loc(i,2)<=Loc_o2(n,2)+ro+ri(i) && Loc(i,2)>=Loc_o2(n,2)-ro-ri(i) && Loc(i,1)<=Loc_o2(n,1)-ro
                diw=Loc_o2(n,1)-ro-Loc(i,1);
                if diw>ri(i)
                    F3(i,:)=F3(i,:)+Ai*exp((ri(i)-diw)/Bi)*niw;
                else
                    F3(i,:)=F3(i,:)+k*(ri(i)-diw)*niw+Ai*exp((ri(i)-diw)/Bi)*niw-kappa*(ri(i)-diw)*sum((Vel(i,:).*tiw))*tiw;
                end
            end
        end
        if (Loc(i,2)<SE(1,2)-gap_safe/2-ri(i) && Loc(i,2)>SE(2,2)+gap_safe/2+ri(i)) || Loc(i,2)>SE(1,2)+gap_safe/2+ri(i) || Loc(i,2)<SE(2,2)-gap_safe/2-ri(i)
            diw=Lx-Loc(i,1);
            if diw>ri(i)
                F3(i,:)=F3(i,:)+Ai*exp((ri(i)-diw)/Bi)*niw;
            else
                F3(i,:)=F3(i,:)+k*(ri(i)-diw)*niw+Ai*exp((ri(i)-diw)/Bi)*niw-kappa*(ri(i)-diw)*sum((Vel(i,:).*tiw))*tiw;
            end
        end
        % calculate F5 with crowd
        for j=1:Num_p
            if i==j||Mar(j)==1
                continue;
            end
            rij=ri(i)+ri(j);
            dij=sqrt(sum((Loc(i,:)-Loc(j,:)).^2));
            nij=(Loc(i,:)-Loc(j,:))/dij;
            tij=[-nij(1,2),nij(1,1)];
            if dij<rij
                F5(i,:)=F5(i,:)+k*(rij-dij)*nij+kappa*(rij-dij)*sum((Vel(j,:)-Vel(i,:)).*tij)*tij;
            end
        end
    end
    % advection by forward euler
    F=F2+F3+F5;
    Vel=Vel+F/mi*dt;
    Loc=Loc+Vel*dt;
    % mark the person who has escaped
    for i=1:Num_p
        if Loc(i,1)>Lx
            Mar(i)=1;
        end
    end
    % save the data
    Record_steps=[steps*dt*ones(Num_p,1),(1:Num_p)',sqrt(sum(Vel.^2, 2)),Loc];
    Record_steps(Mar==1,:)=[];
    Record=[Record;Record_steps];
    % simulation stops if everyone has escapes
    if min(Mar)==1
        break;
    end
    % plot figures
    disp(steps)
    plot(Loc(1:Num_p,1),Loc(1:Num_p,2),'o')
    hold on 
    for i=1:Num_p
        rectangle('Position',[Loc(i,1)-ri(i),Loc(i,2)-ri(i),2*ri(i),2*ri(i)],'Curvature',[1,1],'linewidth',0.5)
    end
    for m=1:Num_o1
        rectangle('Position',[x_u1(m,4),y_u1(m,4),Lx_o1,Ly_o1],'Curvature',[0,0],'EdgeColor','k','LineWidth',1)
    end
    for s=1:Num_o2
        rectangle('Position',[x_u2(s,4),y_u2(s,4),ro*2,ro*2],'Curvature',[0,0],'EdgeColor','k','LineWidth',1)
    end
    time = roundn(steps*dt,-1);
    legend(['Time =',num2str(time),'s'],'Location','NorthWest')
    axis([0,Lx,0,Ly])
    set(gca,'DataAspectRatio',[1 1 1])
    hold off
    pause(0.001)
end