void string2float(string add, float* dest)
{
    char data_char[8000],*char_float;
    const char *a=",";  //Separate datas
    int i=0;
    std::ifstream inidate;
    inidata.open(add);
    if (inidata)    cout<<"file open Successful"<<endl;
    else    cout<<"file open FAIL"<<endl;
    inidata.read(data_char,8000);
    char_float=strtok(data_char, a);
    while(char_float!=NULL)
    {        
        dest[i++] = stof(char_float);
        //cout<<'|'<<dest[i-1]<<endl;
        char_float=strtok(NULL, a);
    }
    inidata.close();
}