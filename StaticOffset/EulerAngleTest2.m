function  Mat2 = EulerAngleTest2(Mat,Static0)

% Remove Offset Angle____________________________________________
LR={'Left' 'Right'};
Co={'PelCo' 'ThiCo' 'TibCo' 'UNTibCo' 'FootCo'};
xyz={'x' 'y' 'z'};
Ang={'EulAngPel' 'EulAngHip' 'EulAngKne' 'EulAngAnk'};

for j=1:length(Co)
    for h=1:length(LR)     %Left,Right
        if isfield(Mat.ThiCo,(LR{h}))
        for t=1:size(Mat.ThiCo.(LR{h}),2)
            if ~isempty(Mat.ThiCo.(LR{h})(t).x)
                if j==1
                    R=[Static0.(Co{j}).x;
                        Static0.(Co{j}).y;
                        Static0.(Co{j}).z];
                    for k=1:length(xyz)
                        for f=1:size(Mat.(Co{j}).(LR{h})(t).(xyz{k}),1)
                            temp=R*Mat.(Co{j}).(LR{h})(t).(xyz{k})(f,:)';
                            Mat2.(Co{j}).(LR{h})(t).(xyz{k})(f,:)=temp;
                            clear temp
                        end
                    end
                else
                    
                    if j==5
                        R=[Static0.(Co{j}).(LR{h}).z;
                            Static0.(Co{j}).(LR{h}).y;
                            -Static0.(Co{j}).(LR{h}).x];
                    else
                        R=[Static0.(Co{j}).(LR{h}).x;
                            Static0.(Co{j}).(LR{h}).y;
                            Static0.(Co{j}).(LR{h}).z];
                    end
                    for k=1:length(xyz)
                        for f=1:size(Mat.(Co{j}).(LR{h})(t).(xyz{k}),1)
                            temp=R*(Mat.(Co{j}).(LR{h})(t).(xyz{k})(f,:))';
                            Mat2.(Co{j}).(LR{h})(t).(xyz{k})(f,:)=temp;
                            clear temp
                        end
                    end
                end
            end
        end
    end
    end
end


% Euler angles calculation____________________________________________
rel=[0 0; 1 2; 2 4; 3 5];
for g=1:length(Ang)
    for h=1:length(LR)
        if isfield(Mat.ThiCo,(LR{h}))
        for t=1:size(Mat.ThiCo.(LR{h}),2)
            if ~isempty(Mat.ThiCo.(LR{h})(t).x)
                if g==1
                    for f=1:size(Mat2.(Co{g}).(LR{h})(t).(xyz{1}),1)
                        R=[Mat2.(Co{g}).(LR{h})(t).x(f,:);
                            Mat2.(Co{g}).(LR{h})(t).y(f,:);
                            Mat2.(Co{g}).(LR{h})(t).z(f,:)];
                        R=R';
                        eul1(f,1)=atan2d(-R(2,3),(1-R(2,3)^2)^.5);
                        eul2(f,1)=atan2d(-R(2,3),-(1-R(2,3)^2)^.5);
                        eul1(f,2)=atan2d(R(1,3),R(3,3));
                        eul2(f,2)=atan2d(-R(1,3),-R(3,3));
                        eul1(f,3)=atan2d(R(2,1),R(2,2));
                        eul2(f,3)=atan2d(-R(2,1),-R(2,2));
                        clear R
                    end
                    Mat2.(Ang{g}).(LR{h})(t).D=[eul1(:,2) eul1(:,1) -eul1(:,3)];
                    clear eul1 eul2
                else
                    
                    for f=1:size(Mat2.(Co{rel(g,2)}).(LR{h})(t).(xyz{1}),1)
                        if g==4
                            R1=[Mat2.(Co{rel(g,2)}).(LR{h})(t).z(f,:);
                                Mat2.(Co{rel(g,2)}).(LR{h})(t).y(f,:);
                                -Mat2.(Co{rel(g,2)}).(LR{h})(t).x(f,:)];
                        else
                            R1=[Mat2.(Co{rel(g,2)}).(LR{h})(t).x(f,:);
                                Mat2.(Co{rel(g,2)}).(LR{h})(t).y(f,:);
                                Mat2.(Co{rel(g,2)}).(LR{h})(t).z(f,:)];
                        end
                        
                        %                             R2=[Mat2.(Co{rel(g,1)}).(LR{h})(t).x(f,:);
                        %                                 Mat2.(Co{rel(g,1)}).(LR{h})(t).y(f,:);
                        %                                 Mat2.(Co{rel(g,1)}).(LR{h})(t).z(f,:)];
                        if g==4
                            sgn=-1;
                        else
                            sgn=1;
                        end
                        %                         R=R1*R2';
                        R=R1';
                        eul1(f,1)=atan2d(-R(2,3),(1-R(2,3)^2)^.5);
                        eul2(f,1)=atan2d(-R(2,3),-(1-R(2,3)^2)^.5);
                        eul1(f,2)=atan2d(R(1,3),R(3,3));
                        eul2(f,2)=atan2d(-R(1,3),-R(3,3));
                        eul1(f,3)=atan2d(R(2,1),R(2,2));
                        eul2(f,3)=atan2d(-R(2,1),-R(2,2));
                        clear R R1 R2
                    end
                    sgnL=[1 1 1;-1 -1 -1;1 -1 -1;-1 -1 -1];
                    sgnR=[1 1 1;-1 1 1;1 1 1;-1 1 1];
                    if h==1
                        Mat2.(Ang{g}).(LR{h})(t).D=[sgnL(g,1)*eul1(:,2) sgnL(g,2)*eul1(:,1) sgnL(g,3)*eul1(:,3)];
                    else
                        Mat2.(Ang{g}).(LR{h})(t).D=[sgnR(g,1)*eul1(:,2) sgnR(g,2)*eul1(:,1) sgnR(g,3)*eul1(:,3)];
                    end
                    clear eul1 eul2
                end
            end
        end
    end
    end
end


% Foot progression angle calculation
for h=1:length(LR)
    if isfield(Mat2.FootCo,(LR{h}))
        if ~isempty(Mat2.FootCo.(LR{h}))
            for j=1:size(Mat2.FootCo.(LR{h}),2)
                if ~isempty(Mat.ThiCo.(LR{h})(j).x)
                    for f=1:size(Mat2.FootCo.(LR{h})(j).x,1)
                        R1=[Mat2.FootCo.(LR{h})(j).z(f,:);
                            Mat2.FootCo.(LR{h})(j).y(f,:);
                            -Mat2.FootCo.(LR{h})(j).x(f,:)];
                        R2=eye(3,3);
                        R=R1*R2';
                        R=R';
                        eul1(f,1)=atan2d(-R(2,3),(1-R(2,3)^2)^.5);
                        eul2(f,1)=atan2d(-R(2,3),-(1-R(2,3)^2)^.5);
                        eul1(f,2)=atan2d(R(1,3),R(3,3));
                        eul2(f,2)=atan2d(-R(1,3),-R(3,3));
                        eul1(f,3)=atan2d(R(2,1),R(2,2));
                        eul2(f,3)=atan2d(-R(2,1),-R(2,2));
                        clear R R1 R2
                    end
                    sgnL=[1 -1 -1];
                    sgnR=[1 1 1];
                    if h==1
                        Mat2.EulFootProgressAngle.(LR{h})(j).D=[sgnL(1)*eul1(:,2) sgnL(2)*eul1(:,1) sgnL(3)*eul1(:,3)];
                    else
                        Mat2.EulFootProgressAngle.(LR{h})(j).D=[sgnR(1)*eul1(:,2) sgnR(2)*eul1(:,1) sgnR(3)*eul1(:,3)];
                    end
                    clear eul1 eul2
                end
            end
        end
    end
end
end

