function [P S] = MedMarkerName(P, S)
%%
    LR={'Left' 'Right'};
    ankm={'LANK_MED' 'RANK_MED'};
    knem={'LKNE_MED' 'RKNE_MED'};
    PP=P; PP.Trajectory.Left=[]; PP.Trajectory.Right=[];
    for h=1:length(LR)
        if ~isempty(P.Trajectory.(LR{h}))
        FN=fieldnames(P.Trajectory.(LR{h})); FN2=FN;
        for j=1:size(P.Trajectory.(LR{h}),2)
            for k=1:length(FN)
                if isempty(findstr(lower(FN{k}),lower('ANK_M')))==0
                    temp=findstr(lower(FN{k}),lower('L'));
                    if isempty(temp)==0
                        if temp(1)==1
                            FN2{k}=ankm{1};
                        end
                    end
                    clear temp
                    temp=findstr(lower(FN{k}),lower('R'));
                    if isempty(temp)==0
                        if temp(1)==1
                            FN2{k}=ankm{2};
                        end
                    end
                    clear temp
                end
                if isempty(findstr(lower(FN{k}),lower('KNE_M')))==0
                    temp=findstr(lower(FN{k}),lower('L'));
                    if isempty(temp)==0
                        if temp(1)==1
                            FN2{k}=knem{1};
                        end
                    end
                    clear temp
                    temp=findstr(lower(FN{k}),lower('R'));
                    if isempty(temp)==0
                        if temp(1)==1
                            FN2{k}=knem{2};
                        end
                    end
                    clear temp
                end
                if isempty(P.Trajectory.(LR{h})(j).(FN{k}))==0
                    if P.Trajectory.(LR{h})(j).LASI(end,1)>P.Trajectory.(LR{h})(j).LASI(1,1)
                        PP.Trajectory.(LR{h})(j).(FN2{k})=P.Trajectory.(LR{h})(j).(FN{k})(:,1:3);
                    else
                        PP.Trajectory.(LR{h})(j).(FN2{k})(:,1:2)=-P.Trajectory.(LR{h})(j).(FN{k})(:,1:2);
                        PP.Trajectory.(LR{h})(j).(FN2{k})(:,3)=P.Trajectory.(LR{h})(j).(FN{k})(:,3);
                    end
                else
                    PP.Trajectory.(LR{h})(j).(FN2{k})=[];
                end
            end
        end
        end
    end
    clear P; P=PP; clear PP
  
    SS(1)=S(1); SS(1).Trajectory=[];
    FN=fieldnames(S(1).Trajectory); FN2=FN;
    if S(1).Trajectory.LASI(1,1)>S(1).Trajectory.LPSI(1,1)
        dr=1;
    else
        dr=-1;
    end
    for k=1:length(FN)
        if isempty(findstr(lower(FN{k}),lower('ANK_M')))==0
            temp=findstr(lower(FN{k}),lower('L'));
            if isempty(temp)==0
            if temp(1)==1
                FN2{k}=ankm{1};
            end
            end
            clear temp
            temp=findstr(lower(FN{k}),lower('R'));
            if isempty(temp)==0
            if temp(1)==1
                FN2{k}=ankm{2};
            end      
            end
            clear temp
        end
        if isempty(findstr(lower(FN{k}),lower('KNE_M')))==0
            temp=findstr(lower(FN{k}),lower('L'));
            if isempty(temp)==0
            if temp(1)==1
                FN2{k}=knem{1};
            end
            end
            clear temp
            temp=findstr(lower(FN{k}),lower('R'));
            if isempty(temp)==0
            if temp(1)==1
                FN2{k}=knem{2};
            end      
            end
            clear temp
        end
        if isempty(S(1).Trajectory.(FN{k}))==0
            if dr==1
            SS(1).Trajectory.(FN2{k})=S(1).Trajectory.(FN{k})(:,1:3);
            else
                SS(1).Trajectory.(FN2{k})(:,1:2)=-S(1).Trajectory.(FN{k})(:,1:2);
                SS(1).Trajectory.(FN2{k})(:,3)=S(1).Trajectory.(FN{k})(:,3);
            end
        else
            SS(1).Trajectory.(FN2{k})=[];
        end
    end
    clear S; S=SS; clear SS
end

