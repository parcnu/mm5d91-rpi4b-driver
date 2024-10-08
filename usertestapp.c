#include <stdio.h>
#include <fcntl.h> 
#include <stdlib.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include "mm5d91_ioctl.h"

unsigned int userbuffer[BUFFER_LENGTH];
int fd;
unsigned char user_buf[BUFFER_LENGTH];
int len;
static int calc = 0;
volatile sig_atomic_t running = 1;  // To stop loop when SIGTERM is received

// Signal handler for kernel signals (SIGUSR1)
void signal_handler(int sig)
{
    int32_t cnt;
    calc++;
    printf("Signal Received %i\n", calc);
    cnt = ioctl(fd, IOCTL_GET_MSG_LEN, (int32_t *) &cnt);
    printf("CNT %i\n", cnt);
    read(fd, user_buf, cnt);
    user_buf[cnt] = '\0';
    for (int i = 0; i < cnt; i++)
        printf("%d\n", user_buf[i]);

    fflush(stdout);  // Flush stdout after printing messages
}

// Signal handler for SIGTERM (to exit program)
void sigterm_handler(int sig)
{
    printf("SIGTERM received, exiting...\n");
    fflush(stdout);  // Flush stdout after printing exit message
    running = 0;
}

int main(int argc, char *argv[])
{
    // Fork to run as a background process
    pid_t pid = fork();
    if (pid < 0) {
        perror("Fork failed");
        exit(EXIT_FAILURE);
    }

    // Parent process exits, child runs in background
    if (pid > 0) {
        printf("Running as background process. PID: %d\n", pid);
        fflush(stdout);  // Flush stdout before parent exits
        exit(EXIT_SUCCESS);
    }

    // Redirect output to a file
    freopen("output.log", "w", stdout);
    freopen("output.log", "w", stderr);

    // Signal handler setup
    signal(SIGUSR1, signal_handler);  // Handle kernel signals
    signal(SIGTERM, sigterm_handler); // Handle SIGTERM to exit
    signal(SIGINT, SIG_IGN);          // Ignore Ctrl+C (SIGINT)

    // Open the mm5d91 device file
    fd = open("/dev/mm5d91", O_RDWR);
    if (fd < 0) {
        perror("fd failed");
        close(fd);
        exit(2);
    }

    // Main loop to handle signals until SIGTERM is received
    while (running) {
        pause();  // Wait for signals
    }

    // Flush any remaining output
    fflush(stdout);
    fflush(stderr);

    // Close the device file
    close(fd);

    // Close the redirected streams
    fclose(stdout);
    fclose(stderr);

    return 0;
}