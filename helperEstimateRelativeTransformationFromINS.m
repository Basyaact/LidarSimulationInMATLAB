function initTform = helperEstimateRelativeTransformationFromINS(insPose, insPosePrev)

location     = [insPose(1) insPose(2) 0];
locationPrev = [insPosePrev(1) insPosePrev(2) 0];

orientation     = [0 0 insPose(3)];
orientationPrev = [0 0 insPosePrev(3)];

tmatCurrToWorld = poseToWorld(location, orientation);
tmatPrevToWorld = poseToWorld(locationPrev, orientationPrev);

tmatInit = tmatPrevToWorld \ tmatCurrToWorld;

initTform = rigid3d(tmatInit');

    function tmat = poseToWorld(loc, orient)
        %------------------------------------
        %TODO Remove trvec2tform, eul2tform
        %------------------------------------
        orient = deg2rad(orient);
        tmat = trvec2tform(loc) * eul2tform(orient(3:-1:1));
    end
end