/* ���: requests.h
 * ������:  ������ hid-custom-rq
 * �����: Christian Starkjohann
 * �������: microsin.ru
 * ���� ��������: 2008-04-09
 * ���������: 4
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * ��������: GNU GPL v2 (��. License.txt) ��� ������������� (CommercialLicense.txt) 
 * �������: $Id: requests.h 553 2008-04-17 19:00:20Z cs $
 */

/* ���� ������������ ���� �������� ����� ����� firmware � �� �����. �� ������
 *  ������ �������� USB numbers (� ����������� ���� ������), ������������ ���
 *  ������������ ����� ������ � ����������� USB.
 */

#ifndef __REQUESTS_H_INCLUDED__
#define __REQUESTS_H_INCLUDED__

#define CUSTOM_RQ_SET_STATUS    1
/* ��������� ��������� LED (���������). Control-OUT.
 * ������������� ������ ���������� � ���� "wValue" ����������� (control)
 *  ��������. ������� ������ OUT �� ����������. ��� 0 �������� ����� wValue 
 *  ��������� LED.
 */

#define CUSTOM_RQ_GET_STATUS    2
/* ��������� ��������� LED. Control-IN.
 * ��� ����������� �������� (control transfer) ��������� 1 ���� ������, ��� 
 *  ���������� ���������� ������� ��������� �����. ��������� ���������� � 
 *  ���� 0 �����.
 */

#define CUSTOM_RQ_SET_R_PWM    	3
#define CUSTOM_RQ_GET_R_PWM    	4
#define CUSTOM_RQ_SET_G_PWM    	5
#define CUSTOM_RQ_GET_G_PWM    	6
#define CUSTOM_RQ_SET_B_PWM    	7
#define CUSTOM_RQ_GET_B_PWM    	8

#endif /* __REQUESTS_H_INCLUDED__ */
