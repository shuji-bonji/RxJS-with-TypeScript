---
description: Der retry-Operator reabonniert die Quelle eine angegebene Anzahl von Malen, wenn ein Fehler im Observable auftritt. Er ist effektiv bei der Wiederherstellung von vorübergehenden Kommunikationsfehlern wie Netzwerkausfällen oder Verarbeitung, bei der ein erneuter Versuch Erfolg haben könnte.
---

# retry - Wiederholung bei Fehler

Der `retry`-Operator **reabonniert das Quell-Observable eine angegebene Anzahl von Malen, wenn ein Fehler auftritt**.
Er eignet sich für **Verarbeitung, bei der ein erneuter Versuch bei Fehlern Erfolg haben könnte**, wie z.B. vorübergehende Netzwerkausfälle.

## 🔰 Grundlegende Syntax und Funktionsweise

### retry(count) - Grundform

```ts
import { throwError, of } from 'rxjs';
import { retry, catchError } from 'rxjs';

throwError(() => new Error('Vorübergehender Fehler'))
  .pipe(
    retry(2), // Maximal 2 Wiederholungen
    catchError((error) => of(`Endfehler: ${error.message}`))
  )
  .subscribe(console.log);
// Ausgabe:
// Endfehler: Vorübergehender Fehler
```

In diesem Beispiel wird nach dem ersten Fehler maximal 2 Mal wiederholt, und wenn alles fehlschlägt, wird die Fallback-Nachricht ausgegeben.

### retry(config) - Konfigurationsobjekt-Form (RxJS 7.4+)

Ab RxJS 7.4 ist durch Übergabe eines Konfigurationsobjekts eine detailliertere Steuerung möglich.

```ts
import { throwError, of } from 'rxjs';
import { retry, catchError, tap } from 'rxjs';

let attemptCount = 0;

throwError(() => new Error('Vorübergehender Fehler'))
  .pipe(
    tap({
      subscribe: () => {
        attemptCount++;
        console.log(`Versuch ${attemptCount}`);
      }
    }),
    retry({
      count: 2,           // Maximal 2 Wiederholungen
      delay: 1000,        // 1 Sekunde warten vor Wiederholung (verwendet intern asyncScheduler)
      resetOnSuccess: true // Zähler bei Erfolg zurücksetzen
    }),
    catchError((error) => of(`Endfehler: ${error.message}`))
  )
  .subscribe(console.log);

// Ausgabe:
// Versuch 1
// Versuch 2
// Versuch 3
// Endfehler: Vorübergehender Fehler
```

> [!NOTE] Steuerung des Retry-Timings
> Bei Angabe der `delay`-Option wird intern **asyncScheduler** verwendet. Für detailliertere Retry-Timing-Steuerung (wie exponentielles Backoff) siehe [Scheduler-Typen und Verwendung - Fehler-Retry-Steuerung](/de/guide/schedulers/types#fehler-retry-steuerung).

[🌐 RxJS Offizielle Dokumentation - retry](https://rxjs.dev/api/index/function/retry)

## 💡 Typische Anwendungsfälle

Das folgende Beispiel zeigt eine Konfiguration mit **zufällig erfolgreichem/fehlgeschlagenem asynchronem Prozess**, der bis zu 3 Mal wiederholt wird.

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs';

let attempt = 0;

interval(1000)
  .pipe(
    mergeMap(() => {
      attempt++;
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        return throwError(() => new Error(`Fehler #${attempt}`));
      } else {
        return of(`Erfolg #${attempt}`);
      }
    }),
    retry(3),
    catchError((err) => of(`Endgültiger Fehler: ${err.message}`))
  )
  .subscribe(console.log);
// Ausgabe:
// Erfolg #1
// Erfolg #5
// Erfolg #6
// Endgültiger Fehler: Fehler #7
```

## 🧪 Praktisches Codebeispiel (mit UI)

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs';

// Ausgabebereich
const retryOutput = document.createElement('div');
retryOutput.innerHTML = '<h3>Beispiel für retry (API-Request-Simulation):</h3>';
document.body.appendChild(retryOutput);

// Request-Status-Anzeige
const requestStatus = document.createElement('div');
requestStatus.style.marginTop = '10px';
requestStatus.style.padding = '10px';
requestStatus.style.border = '1px solid #ddd';
requestStatus.style.maxHeight = '200px';
requestStatus.style.overflowY = 'auto';
retryOutput.appendChild(requestStatus);

// API-Request, der zufällig erfolgreich oder fehlerhaft ist
let attemptCount = 0;

function simulateRequest() {
  attemptCount++;

  const logEntry = document.createElement('div');
  logEntry.textContent = `Versuch #${attemptCount} Request wird gesendet...`;
  requestStatus.appendChild(logEntry);

  return interval(1000).pipe(
    mergeMap(() => {
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        const errorMsg = document.createElement('div');
        errorMsg.textContent = `Versuch #${attemptCount} Fehler: Netzwerkfehler`;
        errorMsg.style.color = 'red';
        requestStatus.appendChild(errorMsg);

        return throwError(() => new Error('Netzwerkfehler'));
      } else {
        const successMsg = document.createElement('div');
        successMsg.textContent = `Versuch #${attemptCount} Erfolg!`;
        successMsg.style.color = 'green';
        requestStatus.appendChild(successMsg);

        return of({ id: 1, name: 'Daten erfolgreich abgerufen' });
      }
    }),
    retry(3),
    catchError((err) => {
      const finalError = document.createElement('div');
      finalError.textContent = `Alle Wiederholungen fehlgeschlagen: ${err.message}`;
      finalError.style.color = 'red';
      finalError.style.fontWeight = 'bold';
      requestStatus.appendChild(finalError);

      return of({ error: true, message: 'Wiederholung fehlgeschlagen' });
    })
  );
}

// Request-Start-Button
const startButton = document.createElement('button');
startButton.textContent = 'Request starten';
startButton.style.padding = '8px 16px';
startButton.style.marginTop = '10px';
retryOutput.insertBefore(startButton, requestStatus);

startButton.addEventListener('click', () => {
  attemptCount = 0;
  requestStatus.innerHTML = '';
  startButton.disabled = true;

  simulateRequest().subscribe((result) => {
    const resultElement = document.createElement('div');
    if ('error' in result) {
      resultElement.textContent = `Endergebnis: ${result.message}`;
      resultElement.style.backgroundColor = '#ffebee';
    } else {
      resultElement.textContent = `Endergebnis: ${result.name}`;
      resultElement.style.backgroundColor = '#e8f5e9';
    }

    resultElement.style.padding = '10px';
    resultElement.style.marginTop = '10px';
    resultElement.style.borderRadius = '5px';
    requestStatus.appendChild(resultElement);

    startButton.disabled = false;
  });
});
```

## ✅ Zusammenfassung

- `retry(n)` wiederholt maximal `n` Mal, wenn das Observable einen Fehler ausgibt
- `retry` wird **bis zum erfolgreichen Abschluss erneut ausgeführt** (bei anhaltenden Fehlern tritt ein Fehler auf)
- Effektiv für **asynchrone APIs und Netzwerk-Requests**, bei denen vorübergehende Ausfälle auftreten
- Üblicherweise in Kombination mit `catchError` zur Angabe von **Fallback-Verarbeitung**
- Ab RxJS 7.4+ können `delay` und `resetOnSuccess` im Konfigurationsobjekt-Format angegeben werden

## Verwandte Seiten

- [retry und catchError](/de/guide/error-handling/retry-catch) - Kombinationsmuster von retry und catchError, praktische Anwendungsbeispiele
- [Debugging von Retries](/de/guide/error-handling/retry-catch#リトライのデバッグ) - Verfolgungsmethoden für Versuchsanzahl (5 Implementierungsmuster)
- [Scheduler-Typen und Verwendung](/de/guide/schedulers/types#エラーリトライの制御) - Detaillierte Steuerung des Retry-Timings, Implementierung von exponentiellem Backoff
- [RxJS Debugging-Techniken](/de/guide/debugging/#シナリオ6-リトライの試行回数を追跡したい) - Retry-Debugging-Szenario
