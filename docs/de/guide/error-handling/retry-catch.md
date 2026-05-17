---
description: "Erklärung robuster Fehlerbehandlungsstrategien durch Kombination der Operatoren retry und catchError. Lernen Sie, wie Sie vorübergehende Störungen wiederholen, exponentielle Backoff-Muster, bedingte Wiederholungen und geeignete Fallback-Verarbeitung typsicher in TypeScript implementieren, anhand praktischer Codebeispiele."
---
# retry und catchError - Effektive Kombination

Detaillierte Erklärung der beiden Kernoperatoren der Fehlerbehandlung in RxJS, `retry` und `catchError`. Durch Kombination dieser können Sie eine robuste Fehlerbehandlungsstrategie realisieren.

## retry - Wiederholung bei Fehler (Grundmuster)

Der `retry`-Operator ist ein Operator, der **die Stream-Ausführung eine bestimmte Anzahl von Malen wiederholt**, wenn ein Fehler im Stream auftritt. Er ist besonders effektiv für Operationen wie Netzwerkanfragen, die vorübergehend fehlschlagen können.

[🌐 RxJS Official Documentation - retry](https://rxjs.dev/api/index/function/retry)

### Grundmuster

```ts
import { Observable, of } from 'rxjs';
import { retry, map } from 'rxjs';

// Funktion, die zufällig Fehler generiert
function getDataWithRandomError(): Observable<string> {
  return of('Daten').pipe(
    map(() => {
      if (Math.random() < 0.7) {
        throw new Error('Zufälliger Fehler aufgetreten');
      }
      return 'Datenabruf erfolgreich!';
    })
  );
}

// Bis zu 3 Wiederholungen
getDataWithRandomError()
  .pipe(retry(3))
  .subscribe({
    next: (data) => console.log('Erfolg:', data),
    error: (err) => console.error('Fehler (nach 3 Wiederholungen):', err.message),
  });

// Ausgabe:
// Erfolg: Datenabruf erfolgreich!
// Fehler (nach 3 Wiederholungen): Zufälliger Fehler aufgetreten ⇦ Wird angezeigt, wenn 3x fehlgeschlagen
```

### Echtzeitüberwachung des Wiederholungsstatus

```ts
import { Observable, of } from 'rxjs';
import { retry, tap, catchError, map } from 'rxjs';

let attempts = 0;

function simulateFlakyRequest(): Observable<string> {
  return of('Anfrage').pipe(
    tap(() => {
      attempts++;
      console.log(`Versuch #${attempts}`);
    }),
    map(() => {
      if (attempts < 3) {
        throw new Error(`Fehler #${attempts}`);
      }
      return 'Erfolg!';
    })
  );
}

simulateFlakyRequest()
  .pipe(
    retry(3),
    catchError((error: unknown) => {
      console.log('Alle Wiederholungen fehlgeschlagen:', (error instanceof Error ? error.message : String(error)));
      return of('Fallback-Wert');
    })
  )
  .subscribe({
    next: (result) => console.log('Endergebnis:', result),
    complete: () => console.log('Abgeschlossen'),
  });

// Ausgabe:
// Versuch #1
// Versuch #2
// Versuch #3
// Endergebnis: Erfolg!
// Abgeschlossen
```

> [!NOTE] Retry-Timing und Scheduler
> Wenn eine Verzögerungszeit mit dem `retry`-Operator angegeben wird (z.B. `retry({ delay: 1000 })`), wird intern **asyncScheduler** verwendet. Durch Nutzung des Schedulers können Sie das Retry-Timing fein steuern oder virtuelle Zeit beim Testen verwenden.
>
> Für Details siehe [Scheduler-Typen und Verwendung - Fehler-Retry-Steuerung](/de/guide/schedulers/types#エラーリトライの制御).

## catchError - Fehlerabfang und alternative Verarbeitung (Grundmuster)

Der `catchError`-Operator fängt Fehler ab, die im Stream aufgetreten sind, und verarbeitet sie, indem er **ein alternatives Observable zurückgibt**. Dadurch kann die Verarbeitung fortgesetzt werden, ohne dass der Stream bei Fehlerauftreten unterbrochen wird.

[🌐 RxJS Official Documentation - catchError](https://rxjs.dev/api/index/function/catchError)

### Grundmuster

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

throwError(() => new Error('API-Aufruffehler')) // Ab RxJS 7, Funktionsform empfohlen
  .pipe(
    catchError((error: unknown) => {
      console.error('Fehler aufgetreten:', (error instanceof Error ? error.message : String(error)));
      return of('Standardwert bei Fehlerauftreten');
    })
  )
  .subscribe({
    next: (value) => console.log('Wert:', value),
    complete: () => console.log('Abgeschlossen'),
  });

// Ausgabe:
// Fehler aufgetreten: API-Aufruffehler
// Wert: Standardwert bei Fehlerauftreten
// Abgeschlossen
```

### Fehler erneut werfen

Wenn Sie den Fehler nach Protokollierung erneut werfen möchten

```ts
import { throwError } from 'rxjs';
import { catchError } from 'rxjs';

throwError(() => new Error('Ursprünglicher Fehler')) // Ab RxJS 7, Funktionsform empfohlen
  .pipe(
    catchError((error: unknown) => {
      console.error('Fehler protokollieren:', (error instanceof Error ? error.message : String(error)));
      // Fehler erneut werfen
      return throwError(() => new Error('Transformierter Fehler'));
    })
  )
  .subscribe({
    next: (value) => console.log('Wert:', value),
    error: (err) => console.error('Finaler Fehler:', err.message),
    complete: () => console.log('Abgeschlossen'),
  });

// Ausgabe:
// Fehler protokollieren: Ursprünglicher Fehler
// Finaler Fehler: Transformierter Fehler
```

## Kombination von retry und catchError

In tatsächlichen Anwendungen ist es üblich, `retry` und `catchError` zu kombinieren. Diese Kombination ermöglicht es, vorübergehende Fehler durch Wiederholung zu lösen und im Falle eines endgültigen Fehlschlags einen Fallback-Wert bereitzustellen.

```ts
import { of, throwError } from 'rxjs';
import { retry, catchError, tap } from 'rxjs';

function fetchData() {
  // Observable, das einen Fehler generiert
  return throwError(() => new Error('Netzwerkfehler')) // Ab RxJS 7, Funktionsform empfohlen
    .pipe(
    // Für Debugging
    tap(() => console.log('Datenabruf versuchen')),
    // Bis zu 3 Wiederholungen
    retry(3),
    // Wenn alle Wiederholungen fehlschlagen
    catchError((error: unknown) => {
      console.error('Alle Wiederholungen fehlgeschlagen:', (error instanceof Error ? error.message : String(error)));
      // Standardwert zurückgeben
      return of({
        error: true,
        data: null,
        message: 'Datenabruf fehlgeschlagen',
      });
    })
  );
}

fetchData().subscribe({
  next: (result) => console.log('Ergebnis:', result),
  complete: () => console.log('Verarbeitung abgeschlossen'),
});

// Ausgabe:
// Alle Wiederholungen fehlgeschlagen: Netzwerkfehler
// Ergebnis: {error: true, data: null, message: 'Datenabruf fehlgeschlagen'}
// Verarbeitung abgeschlossen
```

## Erweiterte Wiederholungsstrategie: retry({ count, delay })

Wenn eine flexiblere Wiederholungsstrategie erforderlich ist, verwenden Sie die **Konfigurationsobjekt-Form** des `retry`-Operators (ab RxJS 7.3). Damit können Sie die Anzahl der Wiederholungen und die Verzögerungslogik deklarativ angeben.

> [!IMPORTANT] retryWhen ist deprecated
> Der alte `retryWhen`-Operator (in v7.3 deprecated, in v8 zur Entfernung vorgesehen) sollte durch die in diesem Abschnitt vorgestellte `retry({ count, delay })`-Form ersetzt werden. Der `delay`-Callback erhält `(error, retryCount)` als Argumente und gibt ein `Observable` (z. B. `timer(ms)`) zurück, um die Wartezeit bis zur nächsten Wiederholung zu steuern.

[🌐 RxJS Official Documentation - retry](https://rxjs.dev/api/index/function/retry)

### Wiederholung mit exponentiellem Backoff

Bei Netzwerkanfragen ist ein exponentielles Backoff-Muster (schrittweise Verlängerung der Wiederholungsintervalle) üblich. Dadurch können Sie die Last auf dem Server reduzieren und gleichzeitig auf die Lösung vorübergehender Probleme warten.

```ts
import { throwError, timer, of } from 'rxjs';
import { retry, catchError } from 'rxjs';

function fetchWithRetry() {
  return throwError(() => new Error('Netzwerkfehler')).pipe(
    // Empfohlen ab RxJS 7.3+: retry({ count, delay })-Form
    retry({
      count: 5, // Bis zu 5 Wiederholungen
      delay: (error, retryCount) => {
        console.log('Fehler aufgetreten:', error.message);
        // Exponentielles Backoff (auf 10 Sekunden begrenzt)
        const delayMs = Math.min(1000 * Math.pow(2, retryCount), 10000);
        console.log(`${retryCount}. Wiederholung in ${delayMs}ms`);
        // timer verwendet intern asyncScheduler
        return timer(delayMs);
      }
    }),
    // Finaler Fallback
    catchError((error: unknown) => {
      console.error('Alle Wiederholungen fehlgeschlagen:', (error instanceof Error ? error.message : String(error)));
      return of({
        error: true,
        message: 'Verbindung fehlgeschlagen. Bitte später erneut versuchen.',
      });
    })
  );
}

fetchWithRetry().subscribe({
  next: (result) => console.log('Ergebnis:', result),
  error: (err) => console.error('Nicht behandelter Fehler:', err),
});

// Ausgabe:
// Fehler aufgetreten: Netzwerkfehler
// 1. Wiederholung in 2000ms
// Fehler aufgetreten: Netzwerkfehler
// 2. Wiederholung in 4000ms
// Fehler aufgetreten: Netzwerkfehler
// 3. Wiederholung in 8000ms
```

> [!TIP] Detaillierte Retry-Steuerung mit Scheduler
> Im obigen Beispiel wird `timer()` verwendet, aber wenn eine erweiterte Steuerung erforderlich ist, können Sie durch explizite Angabe eines Schedulers das Retry-Timing fein anpassen oder virtuelle Zeit beim Testen verwenden.
>
> Für Details siehe [Scheduler-Typen und Verwendung - Fehler-Retry-Steuerung](/de/guide/schedulers/types#エラーリトライの制御).

## Retry-Debugging

Beim Debuggen der Retry-Verarbeitung ist es wichtig, die Anzahl der Versuche und das Ergebnis jedes Versuchs zu verfolgen. Im Folgenden werden praktische Methoden zur Echtzeitüberwachung des Retry-Status vorgestellt.

### Methode 1: tap error-Callback (Grundlegend)

Durch Verwendung des `error`-Callbacks des `tap`-Operators können Sie die Versuchsanzahl bei Fehlerauftreten zählen.

```typescript
import { throwError, of, retry, catchError, tap } from 'rxjs';
let attemptCount = 0;

throwError(() => new Error('Vorübergehender Fehler'))
  .pipe(
    tap({
      error: () => {
        attemptCount++;
        console.log(`Versuchsanzahl: ${attemptCount}`);
      }
    }),
    retry(2),
    catchError((error: unknown) => {
      console.log(`Finale Versuchsanzahl: ${attemptCount}`);
      return of(`Finaler Fehler: ${(error instanceof Error ? error.message : String(error))}`);
    })
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Subscribe-Fehler:', err)
  });

// Ausgabe:
// Versuchsanzahl: 1
// Versuchsanzahl: 2
// Versuchsanzahl: 3
// Finale Versuchsanzahl: 3
// Finaler Fehler: Vorübergehender Fehler
```

> [!NOTE] Einschränkung bei throwError
> `throwError` gibt keinen Wert aus und gibt sofort einen Fehler aus, daher wird der `next`-Callback von `tap` nicht ausgeführt. Sie müssen den `error`-Callback verwenden.

### Methode 2: Detaillierte Verfolgung mit retry({ count, delay }) (Empfohlen)

Für detailliertere Informationen (Versuchsanzahl, Verzögerungszeit, Fehlerinhalt) verwenden Sie die Konfigurationsobjekt-Form von `retry` (ab v7.3, Ersatz für `retryWhen`).

```typescript
import { throwError, of, timer, retry, catchError } from 'rxjs';

throwError(() => new Error('Vorübergehender Fehler'))
  .pipe(
    retry({
      count: 2, // Bis zu 2 Wiederholungen
      delay: (error, retryCount) => {
        console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
        console.log(`🔄 Retry ${retryCount}. Versuch`);
        console.log(`   Fehler: ${error.message}`);

        const delayMs = 1000;
        console.log(`⏳ Wiederholung in ${delayMs}ms...`);
        console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);

        return timer(delayMs);
      }
    }),
    catchError((error: unknown) => {
      console.log(`\nEndergebnis: Alle Retries fehlgeschlagen`);
      return of(`Finaler Fehler: ${(error instanceof Error ? error.message : String(error))}`);
    })
  )
  .subscribe(result => console.log('Ergebnis:', result));

// Ausgabe:
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Retry 1. Versuch
//    Fehler: Vorübergehender Fehler
// ⏳ Wiederholung in 1000ms...
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// (1 Sekunde warten)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Retry 2. Versuch
//    Fehler: Vorübergehender Fehler
// ⏳ Wiederholung in 1000ms...
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// (1 Sekunde warten)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Retry 3. Versuch
//    Fehler: Vorübergehender Fehler
// ❌ Maximale Retry-Anzahl erreicht
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
//
// Endergebnis: Alle Retries fehlgeschlagen
// Ergebnis: Finaler Fehler: Vorübergehender Fehler
```

### Methode 3: Versuchsanzahl mit Custom Observable verfolgen

Bei tatsächlichen API-Anfragen und anderen Observables, die Werte ausgeben, können Sie die Versuchsanzahl mit einem Custom Observable verwalten.

```typescript
import { Observable, of, retry, catchError } from 'rxjs';
let attemptCount = 0;

// Observable, das Versuchsanzahl zählen kann
const retryableStream$ = new Observable(subscriber => {
  attemptCount++;
  console.log(`[Versuch ${attemptCount}]`);

  // Erste 2 Male fehlschlagen, beim 3. Mal erfolgreich
  if (attemptCount < 3) {
    subscriber.error(new Error(`Fehlgeschlagen (Versuch${attemptCount})`));
  } else {
    subscriber.next('Erfolgsdaten');
    subscriber.complete();
  }
});

retryableStream$
  .pipe(
    retry(2),
    catchError((error: unknown) => {
      console.log(`[Abgeschlossen] Insgesamt ${attemptCount} Versuche fehlgeschlagen`);
      return of(`Finaler Fehler: ${(error instanceof Error ? error.message : String(error))}`);
    })
  )
  .subscribe({
    next: data => console.log('[Ergebnis]', data),
    complete: () => console.log('[Abgeschlossen]')
  });

// Ausgabe:
// [Versuch 1]
// [Versuch 2]
// [Versuch 3]
// [Ergebnis] Erfolgsdaten
// [Abgeschlossen]
```

### Methode 4: Exponentielles Backoff und Protokollierung

Praktisches Protokollierungsmuster für tatsächliche API-Anfragen.

```typescript
import { timer, of, retry, catchError, finalize } from 'rxjs';
import { ajax } from 'rxjs/ajax';

function fetchWithRetryLogging(url: string, maxRetries = 3) {
  let startTime = Date.now();

  return ajax.getJSON(url).pipe(
    retry({
      count: maxRetries,
      delay: (error, retryCount) => {
        const elapsed = Date.now() - startTime;

        console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
        console.log(`🔄 Retry-Information`);
        console.log(`   Anzahl: ${retryCount}/${maxRetries}`);
        console.log(`   Fehler: ${error.message || error.status}`);
        console.log(`   Verstrichene Zeit: ${elapsed}ms`);

        // Exponentielles Backoff
        const delayMs = Math.min(1000 * Math.pow(2, retryCount - 1), 10000);
        console.log(`⏳ Wiederholung in ${delayMs}ms...`);
        console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);

        return timer(delayMs);
      }
    }),
    catchError((error: unknown) => {
      const totalTime = Date.now() - startTime;
      console.log(`\n❌ Endgültig fehlgeschlagen (Gesamtzeit: ${totalTime}ms)`);
      return of({ error: true, message: 'Datenabruf fehlgeschlagen' });
    }),
    finalize(() => {
      const totalTime = Date.now() - startTime;
      console.log(`\n✅ Verarbeitung abgeschlossen (Gesamtzeit: ${totalTime}ms)`);
    })
  );
}

// Verwendungsbeispiel
fetchWithRetryLogging('https://jsonplaceholder.typicode.com/users/1').subscribe({
  next: data => console.log('Daten:', data),
  error: err => console.error('Fehler:', err)
});
```

### Methode 5: RxJS 7.4+ retry-Konfigurationsobjekt

Ab RxJS 7.4 können Sie ein Konfigurationsobjekt an `retry` übergeben.

```typescript
import { throwError, of, retry, catchError, tap } from 'rxjs';
let attemptCount = 0;

throwError(() => new Error('Vorübergehender Fehler'))
  .pipe(
    tap({
      subscribe: () => {
        attemptCount++;
        console.log(`Versuch ${attemptCount}`);
      },
      error: (err) => console.log(`Fehler aufgetreten:`, err.message)
    }),
    retry({
      count: 2,
      delay: 1000, // 1 Sekunde warten für Retry (verwendet intern asyncScheduler)
      resetOnSuccess: true
    }),
    catchError((error: unknown) => {
      console.log(`Endgültig fehlgeschlagen (insgesamt ${attemptCount} Versuche)`);
      return of(`Finaler Fehler: ${(error instanceof Error ? error.message : String(error))}`);
    })
  )
  .subscribe(result => console.log('Ergebnis:', result));

// Ausgabe:
// Versuch 1
// Fehler aufgetreten: Vorübergehender Fehler
// Versuch 2
// Fehler aufgetreten: Vorübergehender Fehler
// Versuch 3
// Fehler aufgetreten: Vorübergehender Fehler
// Endgültig fehlgeschlagen (insgesamt 3 Versuche)
// Ergebnis: Finaler Fehler: Vorübergehender Fehler
```

> [!TIP] Empfohlener Ansatz für Retry-Debugging
> - **Während Entwicklung**: Methode 2 (retryWhen) oder Methode 4 (detaillierte Protokolle) optimal
> - **Produktionsumgebung**: Methode 4 als Basis, Protokollsendung an Fehlerüberwachungsdienst hinzufügen
> - **Einfache Fälle**: Methode 1 (tap error) oder Methode 5 (retry-Konfiguration) ausreichend
>
> **Verwandte Informationen**:
> - Für Retry-Timing-Steuerung siehe [Scheduler-Typen und Verwendung - Fehler-Retry-Steuerung](/de/guide/schedulers/types#エラーリトライの制御)
> - Für Gesamtübersicht der Debugging-Techniken siehe [RxJS-Debugging-Techniken - Retry-Versuchsanzahl verfolgen](/de/guide/debugging/#シナリオ6-リトライの試行回数を追跡したい)

## Verwendungsbeispiel in tatsächlichen Anwendungen: API-Anfragen

Beispiel für die Verwendung dieser Operatoren bei tatsächlichen API-Anfragen.

```ts
import { Observable, of } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { retry, catchError, finalize, tap } from 'rxjs';

// Ladestatus
let isLoading = false;

function fetchUserData(userId: string): Observable<any> {
  isLoading = true;

  return ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`).pipe(
    // Request-Debugging
    tap((response) => console.log('API-Antwort:', response)),
    // Netzwerkfehler bis zu 2x wiederholen
    retry(2),
    // Fehlerbehandlung
    catchError((error: unknown) => {
      if (error.status === 404) {
        return of({ error: true, message: 'Benutzer nicht gefunden' });
      } else if (error.status >= 500) {
        return of({ error: true, message: 'Serverfehler aufgetreten' });
      }
      return of({ error: true, message: 'Unbekannter Fehler aufgetreten' });
    }),
    // Immer ausgeführt, unabhängig von Erfolg oder Fehler
    finalize(() => {
      isLoading = false;
      console.log('Laden abgeschlossen');
    })
  );
}

// Verwendungsbeispiel
fetchUserData('123').subscribe({
  next: (data) => {
    if (data.error) {
      // Fehlerinformation anzeigen
      console.error('Fehler:', data.message);
    } else {
      // Daten anzeigen
      console.log('Benutzerdaten:', data);
    }
  },
});

// Ausgabe:
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
// Unbekannter Fehler aufgetreten
// Laden abgeschlossen
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
```

## Best Practices

### Wann retry verwendet werden sollte

- Wenn **vorübergehende Fehler** erwartet werden (Netzwerkverbindungsprobleme usw.)
- **Vorübergehende Serverprobleme** (hohe Last oder Timeouts usw.)
- Bei Fehlern, die **möglicherweise durch Wiederholung gelöst werden können**

### Wann retry nicht verwendet werden sollte

- **Authentifizierungsfehler** (401, 403) - Wiederholung löst das Problem nicht
- **Ressource existiert nicht** (404) - Wiederholung findet sie nicht
- **Validierungsfehler** (400) - Problem liegt in der Anfrage selbst
- **Client-seitige Programmfehler** - Wiederholung ist sinnlos

### Effektive Verwendung von catchError

- **Unterschiedliche Verarbeitung** je nach Fehlertyp
- **Verständliche Nachrichten** für Benutzer bereitstellen
- Bei Bedarf **Fallback-Daten** zurückgeben
- Bei Bedarf **Fehler transformieren**

## Zusammenfassung

Durch Kombination von `retry` und `catchError` ist eine robuste Fehlerbehandlung möglich. Versuchen Sie, vorübergehende Fehler durch Wiederholung zu beheben, und verbessern Sie die Benutzererfahrung, indem Sie dauerhafte Fehler mit angemessener Fallback-Verarbeitung behandeln. In tatsächlichen Anwendungen ist es wichtig, die geeignete Strategie je nach Fehlertyp auszuwählen und einen Fallback-Mechanismus bereitzustellen.

Im nächsten Abschnitt werden der `finalize`-Operator zur Ressourcenfreigabe und die Stream-Abschlussverarbeitung erklärt.
