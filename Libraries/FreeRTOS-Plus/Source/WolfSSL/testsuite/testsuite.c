/* testsuite.c
 *
 * Copyright (C) 2006-2015 wolfSSL Inc.
 *
 * This file is part of wolfSSL. (formerly known as CyaSSL)
 *
 * wolfSSL is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * wolfSSL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA
 */

#ifdef HAVE_CONFIG_H
    #include <config.h>
#endif

#include <wolfssl/wolfcrypt/settings.h>

#include <wolfssl/ssl.h>
#include <wolfssl/test.h>
#include "wolfcrypt/test/test.h"

#ifndef SINGLE_THREADED

#include <wolfssl/openssl/ssl.h>
#include <wolfssl/wolfcrypt/sha256.h>

#include "examples/echoclient/echoclient.h"
#include "examples/echoserver/echoserver.h"
#include "examples/server/server.h"
#include "examples/client/client.h"


void file_test(const char* file, byte* hash);

void simple_test(func_args*);

enum {
    NUMARGS = 3
};

static const char *outputName;

int myoptind = 0;
char* myoptarg = NULL;


#ifndef NO_TESTSUITE_MAIN_DRIVER

    static int testsuite_test(int argc, char** argv);

    int main(int argc, char** argv)
    {
        return testsuite_test(argc, argv);
    }

#endif /* NO_TESTSUITE_MAIN_DRIVER */


int testsuite_test(int argc, char** argv)
{
    func_args server_args;

    tcp_ready ready;
    THREAD_TYPE serverThread;

#ifndef USE_WINDOWS_API
    char tempName[] = "/tmp/output-XXXXXX";
    int len = 18;
    int num = 6;
#else
    char tempName[] = "fnXXXXXX";
    int len = 8;
    int num = 6;
#endif

#ifdef HAVE_CAVIUM
        int ret = OpenNitroxDevice(CAVIUM_DIRECT, CAVIUM_DEV_ID);
        if (ret != 0)
            err_sys("Cavium OpenNitroxDevice failed");
#endif /* HAVE_CAVIUM */

    StartTCP();

    server_args.argc = argc;
    server_args.argv = argv;

    wolfSSL_Init();
#if defined(DEBUG_WOLFSSL) && !defined(HAVE_VALGRIND)
    wolfSSL_Debugging_ON();
#endif

#if !defined(WOLFSSL_TIRTOS)
    if (CurrentDir("testsuite") || CurrentDir("_build"))
        ChangeDirBack(1);
    else if (CurrentDir("Debug") || CurrentDir("Release"))
        ChangeDirBack(3);          /* Xcode->Preferences->Locations->Locations*/
                                   /* Derived Data Advanced -> Custom  */
                                   /* Relative to Workspace, Build/Products */
                                   /* Debug or Release */
#endif

#ifdef WOLFSSL_TIRTOS
    fdOpenSession(Task_self());
#endif

    server_args.signal = &ready;
    InitTcpReady(&ready);

    /* wc_ test */
    wolfcrypt_test(&server_args);
    if (server_args.return_code != 0) return server_args.return_code;
 
    /* Simple wolfSSL client server test */
    simple_test(&server_args);
    if (server_args.return_code != 0) return server_args.return_code;

    /* Echo input wolfSSL client server test */
    start_thread(echoserver_test, &server_args, &serverThread);
    wait_tcp_ready(&server_args);
    {
        func_args echo_args;
        char* myArgv[NUMARGS];

        char argc0[32];
        char argc1[32];
        char argc2[32];

        myArgv[0] = argc0;
        myArgv[1] = argc1;
        myArgv[2] = argc2;

        echo_args.argc = 3;
        echo_args.argv = myArgv;

        /* Create unique file name */
        outputName = mymktemp(tempName, len, num);
        if (outputName == NULL) {
            printf("Could not create unique file name");
            return EXIT_FAILURE;
        }

        strcpy(echo_args.argv[0], "echoclient");
        strcpy(echo_args.argv[1], "input");
        strcpy(echo_args.argv[2], outputName);

        /* Share the signal, it has the new port number in it. */
        echo_args.signal = server_args.signal;

        /* make sure OK */
        echoclient_test(&echo_args);
        if (echo_args.return_code != 0) return echo_args.return_code;

#ifdef WOLFSSL_DTLS
        wait_tcp_ready(&server_args);
#endif
        /* send quit to echoserver */
        echo_args.argc = 2;
        strcpy(echo_args.argv[1], "quit");

        echoclient_test(&echo_args);
        if (echo_args.return_code != 0) return echo_args.return_code;
        join_thread(serverThread);
        if (server_args.return_code != 0) return server_args.return_code;
    }

    /* show ciphers */
    {
        char ciphers[1024];
        XMEMSET(ciphers, 0, sizeof(ciphers));
        wolfSSL_get_ciphers(ciphers, sizeof(ciphers)-1);
        printf("ciphers = %s\n", ciphers);
    }

    /* validate output equals input */
    {
        byte input[SHA256_DIGEST_SIZE];
        byte output[SHA256_DIGEST_SIZE];

        file_test("input",  input);
        file_test(outputName, output);
        remove(outputName);
        if (memcmp(input, output, sizeof(input)) != 0)
            return EXIT_FAILURE;
    }

    wolfSSL_Cleanup();
    FreeTcpReady(&ready);

#ifdef WOLFSSL_TIRTOS
    fdCloseSession(Task_self());
#endif

#ifdef HAVE_CAVIUM
        CspShutdown(CAVIUM_DEV_ID);
#endif
    printf("\nAll tests passed!\n");
    return EXIT_SUCCESS;
}

void simple_test(func_args* args)
{
    THREAD_TYPE serverThread;

    func_args svrArgs;
    char *svrArgv[9];
    char argc0s[32];
    char argc1s[32];
    char argc2s[32];
    char argc3s[32];
    char argc4s[32];
    char argc5s[32];
    char argc6s[32];
    char argc7s[32];
    char argc8s[32];

    func_args cliArgs;
    char *cliArgv[NUMARGS];
    char argc0c[32];
    char argc1c[32];
    char argc2c[32];

    svrArgv[0] = argc0s;
    svrArgv[1] = argc1s;
    svrArgv[2] = argc2s;
    svrArgv[3] = argc3s;
    svrArgv[4] = argc4s;
    svrArgv[5] = argc5s;
    svrArgv[6] = argc6s;
    svrArgv[7] = argc7s;
    svrArgv[8] = argc8s;
    cliArgv[0] = argc0c;
    cliArgv[1] = argc1c;
    cliArgv[2] = argc2c;

    svrArgs.argc = 1;
    svrArgs.argv = svrArgv;
    svrArgs.return_code = 0;
    cliArgs.argc = 1;
    cliArgs.argv = cliArgv;
    cliArgs.return_code = 0;

    strcpy(svrArgs.argv[0], "SimpleServer");
    #if !defined(USE_WINDOWS_API) && !defined(WOLFSSL_SNIFFER)  && \
                                     !defined(WOLFSSL_TIRTOS)
        strcpy(svrArgs.argv[svrArgs.argc++], "-p");
        strcpy(svrArgs.argv[svrArgs.argc++], "0");
    #endif
    #ifdef HAVE_NTRU
        strcpy(svrArgs.argv[svrArgs.argc++], "-d");
        strcpy(svrArgs.argv[svrArgs.argc++], "-n");
        strcpy(svrArgs.argv[svrArgs.argc++], "-c");
        strcpy(svrArgs.argv[svrArgs.argc++], "./certs/ntru-cert.pem");
        strcpy(svrArgs.argv[svrArgs.argc++], "-k");
        strcpy(svrArgs.argv[svrArgs.argc++], "./certs/ntru-key.raw");
    #endif
    /* Set the last arg later, when it is known. */

    args->return_code = 0;
    svrArgs.signal = args->signal;
    start_thread(server_test, &svrArgs, &serverThread);
    wait_tcp_ready(&svrArgs);

    /* Setting the actual port number. */
    strcpy(cliArgs.argv[0], "SimpleClient");
    #ifndef USE_WINDOWS_API
        cliArgs.argc = NUMARGS;
        strcpy(cliArgs.argv[1], "-p");
        snprintf(cliArgs.argv[2], sizeof(argc2c), "%d", svrArgs.signal->port);
    #endif

    client_test(&cliArgs);
    if (cliArgs.return_code != 0) {
        args->return_code = cliArgs.return_code;
        return;
    }
    join_thread(serverThread);
    if (svrArgs.return_code != 0) args->return_code = svrArgs.return_code;
}


void wait_tcp_ready(func_args* args)
{
#if defined(_POSIX_THREADS) && !defined(__MINGW32__)
    pthread_mutex_lock(&args->signal->mutex);

    if (!args->signal->ready)
        pthread_cond_wait(&args->signal->cond, &args->signal->mutex);
    args->signal->ready = 0; /* reset */

    pthread_mutex_unlock(&args->signal->mutex);
#else
    (void)args;
#endif
}


void start_thread(THREAD_FUNC fun, func_args* args, THREAD_TYPE* thread)
{
#if defined(_POSIX_THREADS) && !defined(__MINGW32__)
    pthread_create(thread, 0, fun, args);
    return;
#elif defined(WOLFSSL_TIRTOS)
    /* Initialize the defaults and set the parameters. */
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.arg0 = (UArg)args;
    taskParams.stackSize = 65535;
    *thread = Task_create((Task_FuncPtr)fun, &taskParams, NULL);
    if (*thread == NULL) {
        printf("Failed to create new Task\n");
    }
    Task_yield();
#else
    *thread = (THREAD_TYPE)_beginthreadex(0, 0, fun, args, 0, 0);
#endif
}


void join_thread(THREAD_TYPE thread)
{
#if defined(_POSIX_THREADS) && !defined(__MINGW32__)
    pthread_join(thread, 0);
#elif defined(WOLFSSL_TIRTOS)
    while(1) {
        if (Task_getMode(thread) == Task_Mode_TERMINATED) {
		    Task_sleep(5);
            break;
        }
        Task_yield();
    }
#else
    int res = WaitForSingleObject((HANDLE)thread, INFINITE);
    assert(res == WAIT_OBJECT_0);
    res = CloseHandle((HANDLE)thread);
    assert(res);
#endif
}


void InitTcpReady(tcp_ready* ready)
{
    ready->ready = 0;
    ready->port = 0;
#if defined(_POSIX_THREADS) && !defined(__MINGW32__)
      pthread_mutex_init(&ready->mutex, 0);
      pthread_cond_init(&ready->cond, 0);
#endif
}


void FreeTcpReady(tcp_ready* ready)
{
#if defined(_POSIX_THREADS) && !defined(__MINGW32__)
    pthread_mutex_destroy(&ready->mutex);
    pthread_cond_destroy(&ready->cond);
#else
    (void)ready;
#endif
}


void file_test(const char* file, byte* check)
{
    FILE* f;
    int   i = 0, j, ret;
    Sha256   sha256;
    byte  buf[1024];
    byte  shasum[SHA256_DIGEST_SIZE];

    ret = wc_InitSha256(&sha256);
    if (ret != 0) {
        printf("Can't wc_InitSha256 %d\n", ret);
        return;
    }
    if( !( f = fopen( file, "rb" ) )) {
        printf("Can't open %s\n", file);
        return;
    }
    while( ( i = (int)fread(buf, 1, sizeof(buf), f )) > 0 ) {
        ret = wc_Sha256Update(&sha256, buf, i);
        if (ret != 0) {
            printf("Can't wc_Sha256Update %d\n", ret);
            return;
        }
    }

    ret = wc_Sha256Final(&sha256, shasum);
    if (ret != 0) {
        printf("Can't wc_Sha256Final %d\n", ret);
        return;
    }

    memcpy(check, shasum, sizeof(shasum));

    for(j = 0; j < SHA256_DIGEST_SIZE; ++j )
        printf( "%02x", shasum[j] );

    printf("  %s\n", file);

    fclose(f);
}


#else /* SINGLE_THREADED */


int myoptind = 0;
char* myoptarg = NULL;


int main(int argc, char** argv)
{
    func_args server_args;

    server_args.argc = argc;
    server_args.argv = argv;

    if (CurrentDir("testsuite") || CurrentDir("_build"))
        ChangeDirBack(1);
    else if (CurrentDir("Debug") || CurrentDir("Release"))
        ChangeDirBack(3);          /* Xcode->Preferences->Locations->Locations*/
                                   /* Derived Data Advanced -> Custom  */
                                   /* Relative to Workspace, Build/Products */
                                   /* Debug or Release */

    wolfcrypt_test(&server_args);
    if (server_args.return_code != 0) return server_args.return_code;

    printf("\nAll tests passed!\n");
    return EXIT_SUCCESS;
}


#endif /* SINGLE_THREADED */

