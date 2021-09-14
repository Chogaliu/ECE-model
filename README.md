# ECE-model

This is the code for paper "Modeling the dynamics of pedestrian evacuation in a complex environment", which has been accepted by Physica A: Statistical Mechanics and its Application. Available online 14 September 2021.

## Reference Details
- **Title** : Modeling the dynamics of pedestrian evacuation in a complex environment
- **Abstract** : The study of crowd evacuation dynamics is important for the management of public safety, and crowd evacuation dynamics models can be used to simulate the behaviors of pedestrians during an evacuation. In this paper, we propose a new force model for simulation of the evacuation of complex environments. In comparison to existing models, this model considers more environmental factors to make it applicable to complex emergency environments. By verification using real data, the model is demonstrated to be able to predict evacuation efficiency and better reproduce the microscopic behavior of pedestrians under extreme conditions. We discuss the impact of the presence of different kinds of hazards and different types of obstacles in the environment on pedestrian evacuation performance and put forward suggestions for crowd evacuation management.
https://doi.org/10.1016/j.physa.2021.126426

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
