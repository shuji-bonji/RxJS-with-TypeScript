---
description: "Erklärung umfassender Fehlerbehandlungsstrategien in RxJS. Stellt vor, wie die Operatoren catchError, retry, retryWhen und finalize kombiniert werden, Wiederholungen mit exponentiellem Backoff, Fehlerklassifizierung und angemessene Verarbeitung, globale Error-Handler usw., und wie robuste Fehlerbehandlung in TypeScript implementiert wird."
---
# RxJS-Fehlerbehandlungsstrategien

Die Fehlerbehandlung in RxJS ist ein wichtiger Aspekt der reaktiven Programmierung. Durch Implementierung angemessener Fehlerbehandlung werden die Robustheit und Zuverlässigkeit Ihrer Anwendung verbessert. Dieses Dokument erklärt verschiedene Fehlerbehandlungsstrategien, die in RxJS verwendet werden können.

## Grundmuster

In RxJS behandeln Sie Fehler als Teil des Observable-Lebenszyklus. Die grundlegende Fehlerbehandlung umfasst folgende Methoden:

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

// Observable, das einen Fehler generiert
const error$ = throwError(() => new Error('Fehler aufgetreten')); // Ab RxJS 7, Funktionsform empfohlen

// Grundlegende Fehlerbehandlung
error$
  .pipe(
    catchError((error: unknown) => {
      console.error('Fehler abgefangen:', (error instanceof Error ? error.message : String(error)));
      return of('Fallback-Wert nach Fehler');
    })
  )
  .subscribe({
    next: (value) => console.log('Wert:', value),
    error: (err) => console.error('Nicht behandelter Fehler:', err),
    complete: () => console.log('Abgeschlossen'),
  });

// Ausgabe:
// Fehler abgefangen: Fehler aufgetreten
// Wert: Fallback-Wert nach Fehler
// Abgeschlossen
```

## Verschiedene Fehlerbehandlungsstrategien

### 1. Fehler abfangen und alternativen Wert bereitstellen

Verwenden Sie den `catchError`-Operator, um Fehler abzufangen und einen alternativen Wert oder Stream bereitzustellen.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

const source$ = throwError(() => new Error('Datenabruffehler'));

source$.pipe(
  catchError((error: unknown) => {
    console.error('Fehler aufgetreten:', (error instanceof Error ? error.message : String(error)));
    // Alternative Daten zurückgeben
    return of({ isError: true, data: [], message: 'Standarddaten werden angezeigt' });
  })
).subscribe(data => console.log('Ergebnis:', data));

// Ausgabe:
// Fehler aufgetreten: Datenabruffehler
// Ergebnis: {isError: true, data: Array(0), message: 'Standarddaten werden angezeigt'}
```

### 2. Bei Fehler wiederholen

Verwenden Sie den `retry`-Operator, um den Stream bei Fehlerauftreten zu wiederholen (ab v7.3 wird die Form `retry({ count, delay })` empfohlen und ersetzt das alte `retryWhen`).

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, tap } from 'rxjs';

let attemptCount = 0;

interval(1000).pipe(
  mergeMap(val => {
    if (++attemptCount <= 2) {
      return throwError(() => new Error(`Fehler #${attemptCount}`));
    }
    return of('Erfolg!');
  }),
  tap(() => console.log('Ausführung:', attemptCount)),
  retry(2), // Bis zu 2x wiederholen
).subscribe({
  next: value => console.log('Wert:', value),
  error: err => console.error('Finaler Fehler:', err.message),
});

// Ausgabe:
// Ausführung: 3
// Wert: Erfolg!
// Ausführung: 4
// Wert: Erfolg!
// Ausführung: 5
// ...
```

### 3. Wiederholung mit exponentiellem Backoff

Bei Netzwerkanfragen usw. ist „exponentielles Backoff", das die Wiederholungsintervalle schrittweise verlängert, effektiv.

```ts
import { throwError, timer, of } from 'rxjs';
import { retry, tap, catchError } from 'rxjs';

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
// Fehler aufgetreten: Netzwerkfehler
// 4. Wiederholung in 10000ms
// Fehler aufgetreten: Netzwerkfehler
// 5. Wiederholung in 10000ms
// Alle Wiederholungen fehlgeschlagen: Maximale Wiederholungsanzahl überschritten
// Ergebnis: {error: true, message: 'Verbindung fehlgeschlagen. Bitte später erneut versuchen.'}
```

### 4. Ressourcenfreigabe bei Fehlerauftreten

Verwenden Sie den `finalize`-Operator, um Ressourcen freizugeben, wenn der Stream **durch Abschluss oder Fehler** beendet wird.
finalize ist nicht nur bei Fehlerauftreten wirksam, sondern auch bei normalem Abschluss, wenn Sie zuverlässig Cleanup-Verarbeitung durchführen möchten.

```ts
import { throwError } from 'rxjs';
import { catchError, finalize } from 'rxjs';

let isLoading = true;

throwError(() => new Error('Verarbeitungsfehler'))
  .pipe(
    catchError((error: unknown) => {
      console.error('Fehlerverarbeitung:', (error instanceof Error ? error.message : String(error)));
      return throwError(() => error); // Fehler erneut werfen
    }),
    finalize(() => {
      isLoading = false;
      console.log('Ladestatus zurücksetzen:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Wert:', value),
    error: (err) => console.error('Finaler Fehler:', err.message),
    complete: () => console.log('Abgeschlossen'),
  });

// Ausgabe:
// Fehlerverarbeitung: Verarbeitungsfehler
// Finaler Fehler: Verarbeitungsfehler
// Ladestatus zurücksetzen: false
```

## Fehlerbehandlungsmuster

### Fehlerbehandlung einschließlich UI-Element-Anzeigesteuerung

```ts
import { of, throwError } from 'rxjs';
import { catchError, finalize, tap } from 'rxjs';

function fetchData(shouldFail = false) {
  // Ladeanzeige
  showLoadingIndicator();

  // Datenabruf (Erfolg oder Fehler)
  return (
    shouldFail
      ? throwError(() => new Error('API-Fehler'))
      : of({ name: 'Daten', value: 42 })
  ).pipe(
    tap((data) => {
      // Verarbeitung bei Erfolg
      updateUI(data);
    }),
    catchError((error: unknown) => {
      // UI-Aktualisierung bei Fehler
      showErrorMessage((error instanceof Error ? error.message : String(error)));
      // Leere Daten oder Standardwert zurückgeben
      return of({ name: 'Standard', value: 0 });
    }),
    finalize(() => {
      // Ladeanzeige ausblenden, unabhängig von Erfolg oder Fehler
      hideLoadingIndicator();
    })
  );
}

// Hilfsfunktionen für UI-Operationen
function showLoadingIndicator() {
  console.log('Ladeanzeige');
}
function hideLoadingIndicator() {
  console.log('Ladeanzeige ausgeblendet');
}
function updateUI(data: { name: string; value: number }) {
  console.log('UI aktualisiert:', data);
}
function showErrorMessage(message: any) {
  console.log('Fehler anzeigen:', message);
}

// Verwendungsbeispiel
fetchData(true).subscribe();

// Ausgabe:
// Ladeanzeige
// Fehler anzeigen: API-Fehler
// Ladeanzeige ausgeblendet
```

### Verarbeitung mehrerer Fehlerquellen

```ts
import { forkJoin, of, throwError } from 'rxjs';
import { catchError, map } from 'rxjs';

// Mehrere API-Anfragen simulieren
function getUser() {
  return of({ id: 1, name: 'Taro Yamada' });
}

function getPosts() {
  return throwError(() => new Error('Beitragsabruffehler'));
}

function getComments() {
  return throwError(() => new Error('Kommentarabruffehler'));
}

// Alle Daten abrufen und teilweise Fehler zulassen
forkJoin({
  user: getUser().pipe(
    catchError((error: unknown) => {
      console.error('Benutzerabruffehler:', (error instanceof Error ? error.message : String(error)));
      return of(null); // Bei Fehler null zurückgeben
    })
  ),
  posts: getPosts().pipe(
    catchError((error: unknown) => {
      console.error('Beitragsabruffehler:', (error instanceof Error ? error.message : String(error)));
      return of([]); // Bei Fehler leeres Array zurückgeben
    })
  ),
  comments: getComments().pipe(
    catchError((error: unknown) => {
      console.error('Kommentarabruffehler:', (error instanceof Error ? error.message : String(error)));
      return of([]); // Bei Fehler leeres Array zurückgeben
    })
  ),
})
  .pipe(
    map((result) => ({
      ...result,
      // Flag hinzufügen, das anzeigt, ob teilweise Fehler aufgetreten sind
      hasErrors:
        !result.user ||
        result.posts.length === 0 ||
        result.comments.length === 0,
    }))
  )
  .subscribe((data) => {
    console.log('Endergebnis:', data);

    if (data.hasErrors) {
      console.log(
        'Teilweiser Datenabruf fehlgeschlagen, aber verfügbare Daten werden angezeigt'
      );
    }
  });

// Ausgabe:
// Beitragsabruffehler: Beitragsabruffehler
// Kommentarabruffehler: Kommentarabruffehler
// Endergebnis: {user: {…}, posts: Array(0), comments: Array(0), hasErrors: true}
// Teilweiser Datenabruf fehlgeschlagen, aber verfügbare Daten werden angezeigt
```

## Best Practices für Fehlerbehandlung

1. **Fehler immer abfangen**: Fügen Sie in Observable-Ketten immer Fehlerbehandlung hinzu. Besonders wichtig bei lang laufenden Streams.

2. **Aussagekräftige Fehlermeldungen bereitstellen**: Fehlerobjekte sollten Informationen enthalten, die helfen, Ort und Ursache zu identifizieren.

3. **Ressourcen angemessen freigeben**: Verwenden Sie `finalize`, um sicherzustellen, dass Ressourcen unabhängig von Erfolg oder Fehler freigegeben werden.

4. **Wiederholungsstrategie berücksichtigen**: Besonders bei Netzwerkoperationen verbessert die Implementierung einer angemessenen Wiederholungsstrategie die Zuverlässigkeit.

5. **Benutzerfreundliche Fehlerbehandlung**: Zeigen Sie in der UI keine technischen Fehlermeldungen direkt an, sondern stellen Sie Informationen bereit, die Benutzer verstehen können.

```ts
// Beispiel: Umwandlung in benutzerfreundliche Fehlermeldungen
function getErrorMessage(error: any): string {
  if (error.status === 401) {
    return 'Sitzung abgelaufen. Bitte erneut anmelden.';
  } else if (error.status === 404) {
    return 'Angeforderte Ressource nicht gefunden.';
  } else if (error.status >= 500) {
    return 'Serverfehler aufgetreten. Bitte später erneut versuchen.';
  }
  return 'Unerwarteter Fehler aufgetreten.';
}
```

## Zusammenfassung

Die Fehlerbehandlung in RxJS ist ein wichtiger Teil zur Gewährleistung der Robustheit Ihrer Anwendung. Durch angemessene Kombination von Operatoren wie `catchError`, `retry` und `finalize` können Sie verschiedene Fehlerszenarien bewältigen. Entwerfen Sie eine umfassende Fehlerbehandlungsstrategie, um nicht nur Fehler abzufangen, sondern auch die Benutzererfahrung zu verbessern.

## 🔗 Verwandte Abschnitte

- **[Häufige Fehler und Lösungen](/de/guide/anti-patterns/common-mistakes#9-エラーの握りつぶし)** - Anti-Muster in der Fehlerbehandlung überprüfen
- **[retry und catchError](/de/guide/error-handling/retry-catch)** - Detailliertere Anwendungsmethoden erklärt
