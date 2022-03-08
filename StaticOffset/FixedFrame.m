function fr = FixedFrame(S0)
fr=[];
    FN=fieldnames(S0.Trajectory);
    temp=size(S0.Trajectory.LPSI,1);
    for i=round(.8*temp):round(.85*temp)
        for j=1:length(FN)
            if ~isempty(S0.Trajectory.(FN{j}))
                temp2(j)=S0.Trajectory.(FN{j})(i,4);
            end
        end
        if min(temp2)==1
            fr=i;
            break
        end
        clear temp2
    end    
    if isempty(fr)
        fr=1;
    end
end

