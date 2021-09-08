# ECE-model

This is the code for Modeling the dynamics of pedestrian evacuation in a complex environment. It has been accepted by Physica A: Statistical Mechanics and its Applications on 4 Sep 2021.

## Documentation
- **AppleStore_ECE.m** :
- **AppleStore_SFM.m**: 

## Model Details

ECE model is inspired by social force model proposed by Dirk Helbing in 1995.

![image](https://user-images.githubusercontent.com/80196339/132534525-175bac12-4993-4bde-8e55-854933afd41e.jpeg)

Video address: https://www.foxnews.com/us/new-york-apple-store-gunfire-triggers-panic-video-shows

Using an actual evacuation video under extreme panic as evidence, the characteristics of pedestrian behavior under panic were analyzed, and the ECE model was demonstrated to be accurate for predicting evacuation efficiency. Furthermore, it was found to reproduce the individual movements of pedestrians (detouring behavior and the “parallel to single” phenomenon) better than the social force model. 

## Results  

![image](https://user-images.githubusercontent.com/80196339/132535747-0e7ee2e1-62b1-4bd5-978b-8896604fca45.jpeg)

| Model name | RouteDeviation |
| :---: | :---: | 
| ECE | 1.2660 |
| SFM(v=5) | 2.1243 |
| SFM(v=4) | 1.7170 |
| SFM(v=3) | 1.4529 |
| SFM(v=2) | 1.4888 |
