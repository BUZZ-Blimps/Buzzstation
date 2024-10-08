// DisableStyleWarning.ts

export function disableStyleWarning() {
    const originalWarn = console.warn;

    console.warn = (message, ...args) => {
        if (typeof message === 'string' && message.includes('"textShadow*" style props are deprecated. Use "textShadow".')) {
        return; // Suppress the specific warning
        }
        originalWarn(message, ...args); // Call the original console.warn
    };
}