function [frame1,frame2,inlierframe2,inlierframe1] = SURF_tracking(RGB_img,frame_1,frame_2)

 
            frame_2 = receive(RGB_img);        
            frame1 = readImage(frame_1);
            frame2 = readImage(frame_2);
            grayFrame1 = double(rgb2gray(frame1))/255; 
            grayFrame2 = double(rgb2gray(frame2))/255;                  
            points1 = detectSURFFeatures(grayFrame1);
            points2 = detectSURFFeatures(grayFrame2);
            [features1, validPoints1] = extractFeatures(grayFrame1, points1);
            [features2, validPoints2] = extractFeatures(grayFrame2, points2);        
            indexPairs = matchFeatures(features1, features2);
            matchedPoints1 = validPoints1(indexPairs(:,1));
            matchedPoints2 = validPoints2(indexPairs(:,2));
            [tform,inlierframe2,inlierframe1] = estimateGeometricTransform(matchedPoints2,matchedPoints1,'similarity');
    
end