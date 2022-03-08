function P = RemoveMissedData(P)
    LR={'Left' ,'Right'};
    for h=1:length(LR)
        if ~isempty(P.Trajectory.(LR{h}))
            for j=1:size(P.Trajectory.(LR{h}),2)
                FN=fieldnames(P.Trajectory.(LR{h}));
                for k=1:length(FN)
                    if ~isempty(P.Trajectory.(LR{h})(j).(FN{k}))
                        for f=1:size(P.Trajectory.(LR{h})(j).(FN{k}),1)
                            if P.Trajectory.(LR{h})(j).(FN{k})(f,4)==0
                                P.Trajectory.(LR{h})(j).(FN{k})(f,1:3)=nan;
                            end
                        end
                    end
                end
            end
        end
    end
end

