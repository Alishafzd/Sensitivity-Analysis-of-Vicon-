function theta=Rot_XGlobal(Q4,r1,r2)

        Sac=(Q4.Trajectory.LPSI(r1:r2,:) + Q4.Trajectory.RPSI(r1:r2,:))/2;
        Pel=(Q4.Trajectory.LASI(r1:r2,:) + Q4.Trajectory.RASI(r1:r2,:))/2;

        temp= mean(Pel(:,1:2)-Sac(:,1:2));
        PelDir= round(temp/norm(temp));

        if PelDir(1)==1 && PelDir(2)==0
            theta=0;
        else if PelDir(1)==-1 && PelDir(2)==0
                theta=180;
            else if PelDir(1)==0 && PelDir(2)==-1
                    theta=90;
                else if PelDir(1)==0 && PelDir(2)==1
                        theta=-90;
                    end
                end
            end
        end

end



