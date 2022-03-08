function [S, theta]=RotStatic(S)

theta=0;
Sac=(S(1).Trajectory.LPSI + S(1).Trajectory.RPSI)/2;
Pel=(S(1).Trajectory.LASI + S(1).Trajectory.RASI)/2;

temp= mean(Pel(:,1:2)-Sac(:,1:2));
PelDir= round(temp/norm(temp));

if PelDir(1)==1 && PelDir(2)==0
    theta=0;
else if PelDir(1)==-1 && PelDir(2)==0
        theta=180;
    else if PelDir(1)==0 && PelDir(2)==-1
            theta=-90;
        else if PelDir(1)==0 && PelDir(2)==1
                theta=90;
            end
        end
    end
end
Rot=[cosd(theta) -sind(theta) 0; sind(theta) cosd(theta) 0; 0 0 1];

FN=fieldnames(S(1).Trajectory);
for i=1:length(FN)
    if ~isempty(S(1).Trajectory.(FN{i}))
        for f=1:size(S(1).Trajectory.(FN{i}),1)
            temp=S(1).Trajectory.(FN{i})(f,1:3);
            temp2=Rot*temp';
            S(1).Trajectory.(FN{i})(f,1:3)=temp2';
            clear temp temp2
        end
    end
end

% for i=1:size(fp,1)
%     FP=strcat('ForcePlate',num2str(i));
%     for f=1:size(S(1).(FP).COP,1)
%         temp=S(1).(FP).COP(f,1:3);
%         temp2=Rot*temp';
%         S(1).(FP).COP(f,1:3)=temp2';
%         clear temp temp2
%     end
% end
end


