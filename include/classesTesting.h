class Testing
{
public:
    Testing();

    char get_bob()
    {
        return bob;
    }

protected:
    char bob;
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

float CustomMath::drive_cubic(float input) // hi, bro! I hthink 
{
    float DriveSpeed = pow(input, 3) / pow(100,2);

    return DriveSpeed;
}