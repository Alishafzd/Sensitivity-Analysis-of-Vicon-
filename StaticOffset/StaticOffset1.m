function P2 = StaticOffset1(P0)
S0=P0.Static;
if isfield(P0.Events.Left,'LHS')
    Task=1;
else
    Task=2;
end
    %Use Medial Markers
    LR={'Left' 'Right'};
    Co={'PelCo' 'ThiCo' 'TibCo' 'UNTibCo' 'FootCo'};
    xyz={'x' 'y' 'z'};
    P1=RemoveMissedData(P0);
    [S1, ~]=RotStatic(S0);
    % S1=S0;
    [P, S]= MedMarkerName(P1, S1);
    
LEssMRK={'LASI' 'RASI' 'LPSI' 'RPSI' 'LKNE' 'LKNE_MED' 'LANK' 'LANK_MED' 'RANK_MED' 'LTOE'};
REssMRK={'LASI' 'RASI' 'LPSI' 'RPSI' 'RKNE' 'RKNE_MED' 'RANK' 'RANK_MED' 'LANK_MED' 'RTOE'};

flag=[];
for o=1:length(LEssMRK)
    if isfield(S.Trajectory,(LEssMRK{o}))
        if ~isempty(S.Trajectory.(LEssMRK{o}))
        flag=[flag 1];
        else
            flag=[flag 0];
        end
    else
        flag=[flag 0];
    end
    if isfield(S.Trajectory,(REssMRK{o}))
        if ~isempty(S.Trajectory.(REssMRK{o}))
        flag=[flag 1];
        else
            flag=[flag 0];
        end
    else
        flag=[flag 0];
    end
end
if min(flag)==1
    if Task==3 || Task==4
        tempTR=P0.Trajectory;
        P0.Trajectory=[];
        P0.Trajectory.Left=tempTR;
        P0.Trajectory.Right=tempTR;
    end

    
    % Segments Coordinate Defenition___________________________________________
    [Local_S]=CoordinateStatic1(S);
    [Local_D]=CoordinateTest1(P);
    
    % Rotation Matrix for Static Offset _______________________________________
    fr=FixedFrame(S0);
    for i=1:length(Co)
        for k=1:length(xyz)
            if i==1
                %                 Static0.(Co{i}).(xyz{k})=mean(Static.(Co{i}).(xyz{k}),1);
                Static0.(Co{i}).(xyz{k})=Local_S.(Co{i}).(xyz{k})(fr,:);
            else
                for h=1:length(LR)     %Left,Right
                    %                    Static0.(Co{i}).(LR{h}).(xyz{k})=mean(Static.(Co{i}).(LR{h}).(xyz{k}),1);
                    Static0.(Co{i}).(LR{h}).(xyz{k})=Local_S.(Co{i}).(LR{h}).(xyz{k})(fr,:);
                end
            end
        end
    end
    Static0.PelCo.x=[1 0 0];
    Static0.PelCo.y=[0 1 0];
    Static0.PelCo.z=[0 0 1];
    
    %         Static0.ThiCo.Left.x=[1 0 0];
    %     Static0.ThiCo.Left.y=[0 1 0];
    %     Static0.ThiCo.Left.z=[0 0 1];
    %
    %     Static0.TibCo.Left.Left.x=[1 0 0];
    %     Static0.TibCo.Left.y=[0 1 0];
    %     Static0.TibCo.Left.z=[0 0 1];
    %
    %     Static0.UNTibCo.Left.x=[1 0 0];
    %     Static0.UNTibCo.Left.y=[0 1 0];
    %     Static0.UNTibCo.Left.z=[0 0 1];
    %
    %     Static0.FootCo.Left.x=[1 0 0];
    %     Static0.FootCo.Left.y=[0 1 0];
    %     Static0.FootCo.Left.z=[0 0 1];
    %
    %     Static0.ThiCo.Right.x=[1 0 0];
    %     Static0.ThiCo.Right.y=[0 1 0];
    %     Static0.ThiCo.Right.z=[0 0 1];
    %
    %     Static0.TibCo.Right.Left.x=[1 0 0];
    %     Static0.TibCo.Right.y=[0 1 0];
    %     Static0.TibCo.Right.z=[0 0 1];
    %
    %     Static0.UNTibCo.Right.x=[1 0 0];
    %     Static0.UNTibCo.Right.y=[0 1 0];
    %     Static0.UNTibCo.Right.z=[0 0 1];
    %
    %     Static0.FootCo.Right.x=[0 0 -1];
    %     Static0.FootCo.Right.y=[0 1 0];
    %     Static0.FootCo.Right.z=[1 0 0];
    
    % Euler Angle Calculation__________________________________________________
    %     Static.EulerAngle=EulerAngleStatic(Static,Static0);
    SO1=EulerAngleTest1(Local_D,Static0);
    
    Lang={'LPelvisAngles' 'LHipAngles' 'LKneeAngles' 'LAnkleAngles' 'LFootProgressAngles'};
    Rang={'RPelvisAngles' 'RHipAngles' 'RKneeAngles' 'RAnkleAngles' 'RFootProgressAngles'};
    %ReArrenge Angles
    for h=1:length(LR)
        if h==1
            ANG=Lang;
        else
            ANG=Rang;
        end
        %         FN=fieldnames(P.Kinematics.(LR{h}));
        %         for j=1:length(FN)
        %             SO2.(LR{h}).(FN{j})=[];
        %         end
        if isfield(SO1.EulAngPel,(LR{h}))
            if ~isempty(SO1.EulAngPel.(LR{h}))
                for i=1:size(SO1.EulAngPel.(LR{h}),2)
                    SO2.(LR{h})(i).(ANG{1})=SO1.EulAngPel.(LR{h})(i).D;
                    SO2.(LR{h})(i).(ANG{2})=SO1.EulAngHip.(LR{h})(i).D;
                    SO2.(LR{h})(i).(ANG{3})=SO1.EulAngKne.(LR{h})(i).D;
                    SO2.(LR{h})(i).(ANG{4})=SO1.EulAngAnk.(LR{h})(i).D;
                    SO2.(LR{h})(i).(ANG{5})=SO1.EulFootProgressAngle.(LR{h})(i).D;
                end
            end
        else
            SO2.(LR{h})=[];
        end
    end
    
    if Task==3 || Task==4
        P.Trajectory=[];
        P.Trajectory=tempTR;
        
        P.StaticOffset1=SO2.Left;
        DofR=fieldnames(SO2.Right);
        for i=1:5
            for j= 1:size(SO2.Right,2)
                P.StaticOffset1(j).(DofR{i})= SO2.Right(j).((DofR{i}));
            end
        end
    else
        P.StaticOffset1=SO2;
        P.Trajectory=P0.Trajectory;
    end
    P2=P;
else
    P.StaticOffset1=[];
    P.Trajectory=P0.Trajectory;
    P2=P;
end

end

