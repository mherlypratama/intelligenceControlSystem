float soill = 71.39;

void setsoil()
{
    Serial.println("soil begin ok");
}

void soil()
{
    if (jam == 6 && minute == 1 || jam == 10 && minute == 1 || jam == 13 && minute == 1)
    {
        soill = 73.68;
    }
    else
    {
        soill -= 0.04;
    }
}