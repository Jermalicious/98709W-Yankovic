class Testing
{
public:
    Testing();

    char get_bob()
    {
        return bob;
    }

protected:
    char bob = 'a';
};



Testing::Testing()
{
    bob = 'a';
}



class CustomMath
{
public:

    float drive_cubic(float input);
};

float CustomMath::drive_cubic(float input)
{
    //we use 103^2 so it caps at 95% when innput is 100%
    float DriveSpeed = pow(input, 3) / pow(103,2); 

    return DriveSpeed;
}