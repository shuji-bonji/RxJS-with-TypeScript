---
description: timeout ist ein RxJS Utility-Operator, der einen Fehler wirft, wenn innerhalb einer angegebenen Zeit keine Werte vom Observable emittiert werden. Er eignet sich optimal für reaktive Prozesse mit Zeitbeschränkungen wie API-Request-Timeout-Steuerung, Warten auf Benutzeraktionen oder Stream-Verzögerungserkennung. In Kombination mit catchError können Fallback-Aktionen implementiert werden, und durch TypeScript-Typinferenz ist eine typsichere Timeout-Verarbeitung möglich.
---

# timeout - Timeout-Einstellung

Der `timeout`-Operator **wirft einen Fehler, wenn innerhalb der angegebenen Zeit keine Werte vom Observable emittiert werden**.
Er wird häufig für API-Requests oder Warten auf Benutzeraktionen in reaktiven Prozessen verwendet.


## 🔰 Grundlegende Syntax und Funktionsweise

Wenn kein Timeout erreicht wird, funktioniert es normal; bei Überschreitung einer bestimmten Zeit tritt ein Fehler auf.

```ts
import { of } from 'rxjs';
import { delay, timeout, catchError } from 'rxjs';

of('response')
  .pipe(
    delay(500), // 👈 Bei 1500 wird `Timeout-Fehler: fallback` ausgegeben
    timeout(1000),
    catchError((err) => of('Timeout-Fehler: fallback', err))
  )
  .subscribe(console.log);
// Ausgabe:
// response
```

In diesem Beispiel wird der Wert durch `delay(500)` nach 500ms emittiert und erfüllt die Bedingung von `timeout(1000)`, sodass `'response'` normal angezeigt wird.

Bei `delay(1200)` wird folgender `Timeout-Fehler` ausgegeben:
```sh
Timeout-Fehler: fallback
TimeoutErrorImpl {stack: 'Error\n    at _super (http://localhost:5174/node_mo…s/.vite/deps/chunk-RF6VPQMH.js?v=f6400bce:583:26)', message: 'Timeout has occurred', name: 'TimeoutError', info: {…}}
```

[🌐 RxJS Offizielle Dokumentation - timeout](https://rxjs.dev/api/index/function/timeout)

## 💡 Typische Anwendungsfälle

Das folgende Beispiel zeigt sowohl **ein Muster, bei dem ein Timeout auftritt, wenn der Stream verzögert keine Werte emittiert**, als auch **ein Muster mit normaler Emission**.

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs';

const slow$ = interval(1500).pipe(take(3));
const fast$ = interval(500).pipe(take(3));

fast$
  .pipe(
    timeout(1000),
    catchError((err) => of('fallback: timeout occurred'))
  )
  .subscribe(console.log);

slow$
  .pipe(
    timeout(1000),
    catchError((err) => of('fallback: timeout triggered'))
  )
  .subscribe(console.log);
// Ausgabe:
// 0
// 1
// fallback: timeout triggered
// 2
```


## 🧪 Praktisches Codebeispiel (mit UI)

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs';

// Ausgabebereich
const timeoutOutput = document.createElement('div');
timeoutOutput.innerHTML = '<h3>Beispiel für timeout:</h3>';
document.body.appendChild(timeoutOutput);

// Erfolgsbeispiel für Timeout
const normalStream$ = interval(500).pipe(take(5));

const timeoutSuccess = document.createElement('div');
timeoutSuccess.innerHTML = '<h4>Normaler Stream (kein Timeout):</h4>';
timeoutOutput.appendChild(timeoutSuccess);

normalStream$
  .pipe(
    timeout(1000),
    catchError((err) => {
      const errorMsg = document.createElement('div');
      errorMsg.textContent = `Fehler: ${err.message}`;
      errorMsg.style.color = 'red';
      timeoutSuccess.appendChild(errorMsg);
      return of('Fallback-Wert nach Fehler');
    })
  )
  .subscribe((val) => {
    const item = document.createElement('div');
    item.textContent = `Wert: ${val}`;
    timeoutSuccess.appendChild(item);
  });

// Fehlerbeispiel für Timeout
const slowStream$ = interval(1500).pipe(take(5));

const timeoutError = document.createElement('div');
timeoutError.innerHTML = '<h4>Langsamer Stream (Timeout tritt auf):</h4>';
timeoutOutput.appendChild(timeoutError);

slowStream$
  .pipe(
    timeout(1000),
    catchError((err) => {
      const errorMsg = document.createElement('div');
      errorMsg.textContent = `Fehler: ${err.message}`;
      errorMsg.style.color = 'red';
      timeoutError.appendChild(errorMsg);
      return of('Fallback-Wert nach Timeout');
    })
  )
  .subscribe((val) => {
    const item = document.createElement('div');
    item.textContent = `Wert: ${val}`;
    timeoutError.appendChild(item);
  });
```


## ✅ Zusammenfassung

- `timeout` ist ein Steuerungsoperator, der **einen Fehler ausgibt, wenn innerhalb einer bestimmten Zeit keine Emission erfolgt**
- Effektiv für Timeout-Verarbeitung bei Netzwerk-APIs oder Warten auf UI-Aktionen
- In Kombination mit `catchError` kann ein **Fallback-Verhalten** definiert werden
