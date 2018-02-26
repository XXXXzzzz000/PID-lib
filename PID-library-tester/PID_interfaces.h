#ifndef __PID_INTERFACES_H
#define __PID_INTERFACES_H
#include <stdio.h>
//TODO:
/* print 接口 */
// void PID_Print(char *argv...)
// {
//     return ;
// }
/* println 接口 */
void PID_Println(char * data)
{
    printf("%s\r\n",data);
    return ;
}
void PID_Print(char * data)
{
    printf("%s",data);
    return ;
}
/* millis 接口 */
unsigned long millis(void)
{
    static unsigned long tim;
    return tim++;
}
#endif // !__INTERFACES_H
