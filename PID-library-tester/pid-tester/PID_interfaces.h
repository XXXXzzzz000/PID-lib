#ifndef __PID_INTERFACES_H
#define __PID_INTERFACES_H
//TODO:
/* print 接口 */
// void PID_Print(char *argv...)
// {
//     return ;
// }
/* println 接口 */
void PID_Println(char * data)
{
    (void)data;
    return ;
}
/* millis 接口 */
unsigned long millis(void)
{
    return 0;
}
#endif // !__INTERFACES_H
