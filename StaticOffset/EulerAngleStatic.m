function  Mat2 = EulerAngleStatic(Mat,Static0)

   % Remove Offset Angle____________________________________________
    LR={'Left' 'Right'};
    Co={'PelCo' 'ThiCo' 'TibCo' 'UNTibCo' 'FootCo'};
    xyz={'x' 'y' 'z'};
    Ang={'EulAngPel' 'EulAngHip' 'EulAngKne' 'EulAngAnk'};

    for j=1:length(Co)
        if j==1
            R=[Static0.(Co{j}).x;
                Static0.(Co{j}).y;
                Static0.(Co{j}).z];
            for k=1:length(xyz)
                for f=1:size(Mat.(Co{j}).(xyz{k}),1)
                    temp=R*Mat.(Co{j}).(xyz{k})(f,:)';
                    Mat2.(Co{j}).(xyz{k})(f,:)=temp;
                    clear temp
                end
            end
        else
            for h=1:length(LR)     %Left,Right
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
                    for f=1:size(Mat.(Co{j}).(LR{h}).(xyz{k}),1)
                        temp=R*(Mat.(Co{j}).(LR{h}).(xyz{k})(f,:))';
                        Mat2.(Co{j}).(LR{h}).(xyz{k})(f,:)=temp;
                        clear temp
                    end
                end
            end
        end
    end

    
    
    % Euler angles calculation____________________________________________
    rel=[0 0; 1 2; 2 4; 3 5];
    for g=1:length(Ang)
        if g==1
            for f=1:size(Mat2.(Co{g}).(xyz{1}),1)
                R=[Mat2.(Co{g}).x(f,:);
                    Mat2.(Co{g}).y(f,:);
                    Mat2.(Co{g}).z(f,:)];
                R=R';
                eul1(f,1)=atan2d(-R(2,3),(1-R(2,3)^2)^.5);
                eul2(f,1)=atan2d(-R(2,3),-(1-R(2,3)^2)^.5);
                eul1(f,2)=atan2d(R(1,3),R(3,3));
                eul2(f,2)=atan2d(-R(1,3),-R(3,3));
                eul1(f,3)=atan2d(R(2,1),R(2,2));
                eul2(f,3)=atan2d(-R(2,1),-R(2,2));
                clear R
            end
            Mat2.(Ang{g})=[eul1(:,2) eul1(:,1) -eul1(:,3)];
            clear eul1 eul2
        else
            for h=1:length(LR)
                for f=1:size(Mat2.(Co{rel(g,2)}).(LR{h}).(xyz{1}),1)
                    if g==4
                        R1=[Mat2.(Co{rel(g,2)}).(LR{h}).z(f,:);
                            Mat2.(Co{rel(g,2)}).(LR{h}).y(f,:);
                            -Mat2.(Co{rel(g,2)}).(LR{h}).x(f,:)];
                    else
                        R1=[Mat2.(Co{rel(g,2)}).(LR{h}).x(f,:);
                            Mat2.(Co{rel(g,2)}).(LR{h}).y(f,:);
                            Mat2.(Co{rel(g,2)}).(LR{h}).z(f,:)];
                    end
                    if g==2
                        R2=[Mat2.(Co{rel(g,1)}).x(f,:);
                            Mat2.(Co{rel(g,1)}).y(f,:);
                            Mat2.(Co{rel(g,1)}).z(f,:)];
                    else
                        R2=[Mat2.(Co{rel(g,1)}).(LR{h}).x(f,:);
                            Mat2.(Co{rel(g,1)}).(LR{h}).y(f,:);
                            Mat2.(Co{rel(g,1)}).(LR{h}).z(f,:)];
                    end
                    if g==4
                        sgn=-1;
                    else
                        sgn=1;
                    end
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
                sgnL=[1 1 1;-1 -1 -1;1 -1 -1;-1 -1 -1];
                sgnR=[1 1 1;-1 1 1;1 1 1;-1 1 1];
                if h==1  
                    Mat2.(Ang{g}).(LR{h})=[sgnL(g,1)*eul1(:,2) sgnL(g,2)*eul1(:,1) sgnL(g,3)*eul1(:,3)];
                else
                    Mat2.(Ang{g}).(LR{h})=[sgnR(g,1)*eul1(:,2) sgnR(g,2)*eul1(:,1) sgnR(g,3)*eul1(:,3)];
                end
                clear eul1 eul2
            end
        end
    end

end

