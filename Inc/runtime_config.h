#ifndef RUNTIME_CONFIG_H
#define RUNTIME_CONFIG_H

// C runtime
#define PRINT_MESSAGE_ABOUT_HEAP_AND_STACK_COLLISION
#define PRINT_MESSAGES_ABOUT_SHUTDOWN
//#define PRINT_MEMORY_LEFT_AFTER_SBRK

//#define DEFINE_ABORT_FN // Sometimes it is present, sometines no

//! ��������� ������� ��� ������ ���"���, ���� ��������� STL
//! ������ �����, �� �, ��������� ���� �������� -- ��������
//! ������ ������, ��� ����� ��������.
//! �����, ���� �������� ���������� ���������! � ���� ����������
//! (������� �����������) ��������, �� malloc ���� ���� ���������,
//! ��� ��� ���� �������� ����������!
//#define ALLOW_CBRK_TO_RETURN_LESS_MEMORY_THAN_REQUESTED

#endif
