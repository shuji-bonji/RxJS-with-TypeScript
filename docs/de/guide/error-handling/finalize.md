---
description: "Erklärung, wie man mit finalize und complete die Stream-Abschlussverarbeitung und Ressourcenfreigabe in RxJS effektiv durchführt. Stellt praktische Muster vor wie Speicherleck-Prävention, Dateihandle-Freigabe, WebSocket-Verbindungs-Cleanup, UI-Status-Reset usw. Erläutert auch die Unterschiede zur finally-Klausel."
---
# finalize und complete - Ressourcen-Cleanup

In RxJS ist es wichtig, die Stream-Beendigung und Ressourcenfreigabe angemessen zu verwalten. Diese Seite erklärt den Mechanismus des `finalize`-Operators und der `complete`-Benachrichtigung.

## finalize - Operator zur Ressourcenfreigabe

Der `finalize`-Operator ist ein Operator, der den angegebenen Cleanup-Code ausführt, wenn ein Observable **entweder durch Abschluss, Fehler oder Abmeldung beendet wird**.
finalize wird **immer einmal bei Stream-Beendigung** aufgerufen und wird nie mehrfach aufgerufen.

[🌐 RxJS Official Documentation - finalize](https://rxjs.dev/api/index/function/finalize)

### Grundlegende Verwendung von finalize

```ts
import { of } from 'rxjs';
import { finalize, tap } from 'rxjs';

// Variable zur Verwaltung des Ladestatus
let isLoading = true;

// Erfolgreicher Stream
of('Daten')
  .pipe(
    tap((data) => console.log('Datenverarbeitung:', data)),
    // Wird in allen Fällen ausgeführt: Erfolg, Fehler oder Abbruch
    finalize(() => {
      isLoading = false;
      console.log('Ladestatus zurückgesetzt:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Wert:', value),
    complete: () => console.log('Abgeschlossen'),
  });

// Ausgabe:
// Datenverarbeitung: Daten
// Wert: Daten
// Abgeschlossen
// Ladestatus zurückgesetzt: false
```

### finalize bei Fehlerauftreten

```ts
import { throwError } from 'rxjs';
import { finalize, catchError } from 'rxjs';

let isLoading = true;

throwError(() => new Error('Datenabruffehler'))
  .pipe(
    catchError((err) => {
      console.error('Fehlerverarbeitung:', err.message);
      throw err; // Fehler erneut werfen
    }),
    finalize(() => {
      isLoading = false;
      console.log('Ressourcenfreigabe nach Fehler:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Wert:', value),
    error: (err) => console.error('Fehler beim Subscriber:', err.message),
    complete: () => console.log('Abgeschlossen'), // Wird bei Fehler nicht aufgerufen
  });

// Ausgabe:
// Fehlerverarbeitung: Datenabruffehler
// Fehler beim Subscriber: Datenabruffehler
// Ressourcenfreigabe nach Fehler: false
```

### finalize bei Abmeldung

```ts
import { interval } from 'rxjs';
import { finalize } from 'rxjs';

let resource = 'Aktiv';

// Zählt jede Sekunde
const subscription = interval(1000)
  .pipe(
    finalize(() => {
      resource = 'Freigegeben';
      console.log('Ressourcenstatus:', resource);
    })
  )
  .subscribe((count) => {
    console.log('Zählung:', count);

    // Nach 3 Zählungen manuell abmelden
    if (count >= 2) {
      subscription.unsubscribe();
    }
  });

// Ausgabe:
// Zählung: 0
// Zählung: 1
// Zählung: 2
// Ressourcenstatus: Freigegeben
```

finalize ist nicht nur bei Fehlerauftreten wirksam, sondern auch bei normalem Abschluss und manueller Abmeldung (unsubscribe), wenn Sie zuverlässig Cleanup-Verarbeitung durchführen möchten.

## complete - Benachrichtigung über normale Stream-Beendigung

Wenn ein Observable normal beendet wird, wird der `complete`-Callback des Observers aufgerufen. Dies ist der letzte Schritt im Lebenszyklus eines Observable.

### Automatisches complete

Einige Observables werden automatisch abgeschlossen, wenn bestimmte Bedingungen erfüllt sind.

```ts
import { of } from 'rxjs';
import { take } from 'rxjs';

// Endliche Sequenzen werden automatisch abgeschlossen
of(1, 2, 3).subscribe({
  next: (value) => console.log('Wert:', value),
  complete: () => console.log('Endlicher Stream abgeschlossen'),
});

// interval + take begrenzt Stream
interval(1000)
  .pipe(
    take(3) // Nach 3 Werten abgeschlossen
  )
  .subscribe({
    next: (value) => console.log('Zählung:', value),
    complete: () => console.log('Begrenzter Stream abgeschlossen'),
  });

// Ausgabe:
// Wert: 1
// Wert: 2
// Wert: 3
// Endlicher Stream abgeschlossen
// Zählung: 0
// Zählung: 1
// Zählung: 2
// Begrenzter Stream abgeschlossen
```

### Manuelles complete

Bei Subject oder benutzerdefinierten kann complete manuell aufgerufen werden.

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

subject.subscribe({
  next: (value) => console.log('Wert:', value),
  complete: () => console.log('Subject abgeschlossen'),
});

subject.next(1);
subject.next(2);
subject.complete(); // Manuell abschließen
subject.next(3); // Nach Abschluss ignoriert

// Ausgabe:
// Wert: 1
// Wert: 2
// Subject abgeschlossen
```

## Unterschied zwischen finalize und complete

Verstehen Sie die wichtigen Unterschiede.

1. **Ausführungszeitpunkt**
   - `complete`: Wird **nur bei normalem Abschluss** des Observable aufgerufen
   - `finalize`: Wird aufgerufen, wenn Observable **durch Abschluss, Fehler oder Abmeldung** beendet wird

2. **Verwendungszweck**
   - `complete`: Benachrichtigung über normalen Abschluss erhalten (Verarbeitung bei Erfolg)
   - `finalize`: Ressourcenfreigabe und Cleanup zuverlässig durchführen (immer auszuführende Verarbeitung unabhängig von Erfolg oder Fehler)

## Praktische Anwendungsfälle

### API-Aufruf und Ladestatus-Verwaltung

```ts
import { ajax } from 'rxjs/ajax';
import { finalize, catchError } from 'rxjs';
import { of } from 'rxjs';

// Ladestatus
let isLoading = false;

function fetchData(id: string) {
  // Laden beginnen
  isLoading = true;
  const loading = document.createElement('p');
  loading.style.display = 'block';
  document.body.appendChild(loading);
  // document.getElementById('loading')!.style.display = 'block';

  // API-Anfrage
  return ajax.getJSON(`https://jsonplaceholder.typicode.com/posts/${id}`).pipe(
    catchError((error) => {
      console.error('API-Fehler:', error);
      return of({ error: true, message: 'Datenabruf fehlgeschlagen' });
    }),
    // Laden unabhängig von Erfolg oder Fehler beenden
    finalize(() => {
      isLoading = false;
      loading!.style.display = 'none';
      console.log('Ladestatus-Reset abgeschlossen');
    })
  );
}

// Verwendungsbeispiel
fetchData('123').subscribe({
  next: (data) => console.log('Daten:', data),
  complete: () => console.log('Datenabruf abgeschlossen'),
});

// Ausgabe:
//  API-Fehler: AjaxErrorImpl {message: 'ajax error', name: 'AjaxError', xhr: XMLHttpRequest, request: {…}, status: 0, …}
//  Daten: {error: true, message: 'Datenabruf fehlgeschlagen'}
//  Datenabruf abgeschlossen
//  Ladestatus-Reset abgeschlossen
//   GET https://jsonplaceholder.typicode.com/posts/123 net::ERR_NAME_NOT_RESOLVED
```

### Ressourcen-Cleanup

```ts
import { interval } from 'rxjs';
import { finalize, takeUntil } from 'rxjs';
import { Subject } from 'rxjs';

class ResourceManager {
  private destroy$ = new Subject<void>();
  private timerId: number | null = null;

  constructor() {
    // Ressourceninitialisierung
    this.timerId = window.setTimeout(() => console.log('Timer ausgeführt'), 10000);

    // Periodische Verarbeitung
    interval(1000)
      .pipe(
        // Stoppen bei Komponentenvernichtung
        takeUntil(this.destroy$),
        // Ressourcenfreigabe sicherstellen
        finalize(() => {
          console.log('Intervall gestoppt');
        })
      )
      .subscribe((count) => {
        console.log('Wird ausgeführt...', count);
      });
  }

  dispose() {
    // Vernichtungsverarbeitung
    if (this.timerId) {
      window.clearTimeout(this.timerId);
      this.timerId = null;
    }

    // Stream-Stoppsignal
    this.destroy$.next();
    this.destroy$.complete();

    console.log('Ressourcenmanager-Vernichtung abgeschlossen');
  }
}

// Verwendungsbeispiel
const manager = new ResourceManager();

// Nach 5 Sekunden vernichten
setTimeout(() => {
  manager.dispose();
}, 5000);

// Ausgabe:
// Wird ausgeführt... 0
// Wird ausgeführt... 1
// Wird ausgeführt... 2
// Wird ausgeführt... 3
// Wird ausgeführt... 4
// Intervall gestoppt
// Ressourcenmanager-Vernichtung abgeschlossen
```

[📘 RxJS Official: takeUntil()](https://rxjs.dev/api/index/function/takeUntil)

## Best Practices

1. **Ressourcen immer freigeben**: `finalize` verwenden, um Cleanup bei Stream-Beendigung zu garantieren
2. **Ladestatus-Verwaltung**: `finalize` verwenden, um Ladestatus immer zurückzusetzen
3. **Komponenten-Lebenszyklus-Verwaltung**: `takeUntil` und `finalize` kombinieren, um Ressourcen bei Komponentenvernichtung zu bereinigen (besonders empfohlen in Angular usw.)
4. **Verwendung mit Fehlerbehandlung**: `catchError` und `finalize` kombinieren, um Fallback-Verarbeitung nach Fehler und zuverlässiges Cleanup zu realisieren
5. **Abschlussstatus verstehen**: `complete`-Callback verwenden, um zu beurteilen, ob Stream normal abgeschlossen wurde

## Zusammenfassung

`finalize` und `complete` sind wichtige Werkzeuge für Ressourcenverwaltung und Verarbeitungsabschluss in RxJS. `finalize` ist optimal für Ressourcenfreigabe, da es unabhängig davon, wie der Stream endet, zuverlässig ausgeführt wird. Andererseits wird `complete` verwendet, wenn Sie Verarbeitung bei normalem Abschluss durchführen möchten. Durch angemessene Kombination dieser können Sie Speicherlecks verhindern und zuverlässige Anwendungen erstellen.
