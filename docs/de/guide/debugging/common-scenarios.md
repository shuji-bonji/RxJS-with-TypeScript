---
description: "6 häufige RxJS-Debugging-Szenarien: keine Werte, unerwartete Ausgabewerte, nicht abgeschlossene Subscriptions, Memory Leaks, übersehene Fehler, Retry-Verfolgung. Praktische Code-Beispiele zur Identifikation von Ursachen und Lösungsmethoden für häufige Probleme in der Entwicklungspraxis."
---

# Häufige Debugging-Szenarien

Erklärung typischer Probleme in der RxJS-Entwicklung und deren Lösungen mit konkreten Code-Beispielen.

## Szenario 1: Keine Werte fließen

- **Symptom**: Trotz `subscribe` wird kein einziger Wert ausgegeben

### Ursache 1: Vergessene Subscription bei Cold Observable

```ts
import { interval } from 'rxjs';
import { map } from 'rxjs';

// ❌ Ohne Subscription wird nichts ausgeführt
const numbers$ = interval(1000).pipe(
  map(x => {
    console.log('Diese Zeile wird nicht ausgeführt');
    return x * 2;
  })
);

// ✅ Durch Subscription wird es ausgeführt
numbers$.subscribe(value => console.log('Wert:', value));
```

### Ursache 2: Abgeschlossenes Subject

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

subject.complete(); // Abschluss

// ❌ Subscription nach Abschluss empfängt keine Werte
subject.subscribe(value => console.log('Diese Zeile wird nicht ausgeführt'));

// ✅ Subscription vor Abschluss
const subject2 = new Subject<number>();
subject2.subscribe(value => console.log('Wert:', value));
subject2.next(1); // Wert: 1
subject2.complete();
```

### Ursache 3: Filterung mit falschen Bedingungen

```ts
import { of } from 'rxjs';
import { filter, tap } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(
    tap(value => console.log('Vor filter:', value)),
    filter(x => x > 10), // Alle werden herausgefiltert
    tap(value => console.log('Nach filter:', value)) // Diese Zeile wird nicht ausgeführt
  )
  .subscribe({
    next: value => console.log('Endwert:', value),
    complete: () => console.log('Abschluss (keine Werte)')
  });

// Ausgabe:
// Vor filter: 1
// Vor filter: 2
// Vor filter: 3
// Vor filter: 4
// Vor filter: 5
// Abschluss (keine Werte)
```

### Debug-Technik
```ts
import { of, EMPTY } from 'rxjs';
import { filter, tap, defaultIfEmpty } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(
    tap(value => console.log('🔵 Eingabe:', value)),
    filter(x => x > 10),
    tap(value => console.log('🟢 Filter passiert:', value)),
    defaultIfEmpty('Keine Werte') // Standard bei fehlenden Werten
  )
  .subscribe(value => console.log('✅ Ausgabe:', value));

// Ausgabe:
// 🔵 Eingabe: 1
// 🔵 Eingabe: 2
// 🔵 Eingabe: 3
// 🔵 Eingabe: 4
// 🔵 Eingabe: 5
// ✅ Ausgabe: Keine Werte
```

## Szenario 2: Unerwartete Werte werden ausgegeben

- **Symptom**: Es werden andere Werte als erwartet ausgegeben

### Ursache 1: Falsche Reihenfolge der Operatoren

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

// ❌ Ergebnis weicht von der Erwartung ab
of(1, 2, 3, 4, 5)
  .pipe(
    map(x => x * 2),     // 2, 4, 6, 8, 10
    filter(x => x < 5)   // Nur 2, 4 passieren
  )
  .subscribe(value => console.log('Ergebnis:', value));
// Ausgabe: 2, 4

// ✅ Richtige Reihenfolge
of(1, 2, 3, 4, 5)
  .pipe(
    filter(x => x < 5),  // Nur 1, 2, 3, 4 passieren
    map(x => x * 2)      // 2, 4, 6, 8
  )
  .subscribe(value => console.log('Ergebnis:', value));
// Ausgabe: 2, 4, 6, 8
```

### Ursache 2: Unbeabsichtigte Änderung durch gemeinsame Referenz

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const user: User = { id: 1, name: 'Alice' };

of(user)
  .pipe(
    // ❌ Direktes Ändern des Originalobjekts
    map(u => {
      u.name = 'Bob'; // Originalobjekt wird geändert
      return u;
    })
  )
  .subscribe(value => console.log('Nach Änderung:', value));

console.log('Originalobjekt:', user); // { id: 1, name: 'Bob' }

// ✅ Neues Objekt erstellen
of(user)
  .pipe(
    map(u => ({ ...u, name: 'Charlie' })) // Neues Objekt mit Spread-Syntax
  )
  .subscribe(value => console.log('Nach Änderung:', value));

console.log('Originalobjekt:', user); // { id: 1, name: 'Alice' } (nicht geändert)
```

### Ursache 3: Timing asynchroner Verarbeitung

```ts
import { of, delay } from 'rxjs';
import { mergeMap, tap } from 'rxjs';

// ❌ Wartet nicht auf Abschluss asynchroner Verarbeitung
of(1, 2, 3)
  .pipe(
    tap(value => console.log('Start:', value)),
    mergeMap(value =>
      of(value * 2).pipe(
        delay(100 - value * 10) // Größere Werte werden schneller abgeschlossen
      )
    )
  )
  .subscribe(value => console.log('Abschluss:', value));

// Ausgabe:
// Start: 1
// Start: 2
// Start: 3
// Abschluss: 3  ← Kürzeste Verzögerung
// Abschluss: 2
// Abschluss: 1  ← Längste Verzögerung

// ✅ Reihenfolge garantieren
import { concatMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    tap(value => console.log('Start:', value)),
    concatMap(value =>  // mergeMap → concatMap
      of(value * 2).pipe(delay(100 - value * 10))
    )
  )
  .subscribe(value => console.log('Abschluss:', value));

// Ausgabe:
// Start: 1
// Abschluss: 1
// Start: 2
// Abschluss: 2
// Start: 3
// Abschluss: 3
```

## Szenario 3: Subscription wird nicht abgeschlossen (unendlicher Stream)

- **Symptom**: `complete` wird nicht aufgerufen und der Stream endet nicht

```ts
import { interval } from 'rxjs';
import { tap } from 'rxjs';

// ❌ interval gibt unendlich Werte aus
interval(1000)
  .pipe(
    tap(value => console.log('Wert:', value))
  )
  .subscribe({
    complete: () => console.log('Diese Zeile wird nicht ausgeführt')
  });

// ✅ Explizit mit take abschließen
import { take } from 'rxjs';

interval(1000)
  .pipe(
    take(5), // Nach 5 Werten abschließen
    tap(value => console.log('Wert:', value))
  )
  .subscribe({
    complete: () => console.log('Abschluss')
  });
```

### Debug-Technik
```ts
import { interval, timer } from 'rxjs';
import { tap, takeUntil } from 'rxjs';

// Timeout zum Debuggen setzen
const stop$ = timer(5000); // Nach 5 Sekunden abschließen

interval(1000)
  .pipe(
    tap({
      next: value => console.log('Wert:', value),
      complete: () => console.log('Durch Timeout gestoppt')
    }),
    takeUntil(stop$)
  )
  .subscribe();
```

## Szenario 4: Memory Leak (vergessenes Unsubscribe)

- **Symptom**: Die Anwendung wird allmählich langsamer

### Ursache: Nicht mehr benötigte Subscriptions werden nicht aufgehoben

```ts
import { interval } from 'rxjs';

class UserComponent {
  private subscription: any;

  ngOnInit() {
    // ❌ Unsubscribe vergessen
    interval(1000).subscribe(value => {
      console.log('Wert:', value); // Wird auch nach Zerstörung der Komponente weiter ausgeführt
    });
  }

  ngOnDestroy() {
    // Unsubscribe wird nicht durchgeführt
  }
}

// ✅ Subscription ordnungsgemäß verwalten
class UserComponentFixed {
  private subscription: any;

  ngOnInit() {
    this.subscription = interval(1000).subscribe(value => {
      console.log('Wert:', value);
    });
  }

  ngOnDestroy() {
    // Unsubscribe beim Zerstören der Komponente
    if (this.subscription) {
      this.subscription.unsubscribe();
    }
  }
}
```

**Empfohlenes Pattern: `takeUntil` verwenden**

```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class UserComponentBest {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    // ✅ Automatisches Unsubscribe mit takeUntil
    interval(1000)
      .pipe(
        takeUntil(this.destroy$)
      )
      .subscribe(value => console.log('Wert:', value));
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

### Erkennung von Memory Leaks

```ts
import { interval } from 'rxjs';
import { tap } from 'rxjs';

let subscriptionCount = 0;

const trackSubscriptions = <T>() =>
  tap<T>({
    subscribe: () => {
      subscriptionCount++;
      console.log('📈 Subscription-Anzahl:', subscriptionCount);
    },
    unsubscribe: () => {
      subscriptionCount--;
      console.log('📉 Subscription-Anzahl:', subscriptionCount);
    }
  });

// Verwendungsbeispiel
const stream$ = interval(1000).pipe(
  trackSubscriptions()
);

const sub1 = stream$.subscribe();
// Ausgabe: 📈 Subscription-Anzahl: 1

const sub2 = stream$.subscribe();
// Ausgabe: 📈 Subscription-Anzahl: 2

setTimeout(() => {
  sub1.unsubscribe();
  // Ausgabe: 📉 Subscription-Anzahl: 1
}, 3000);
```

## Szenario 5: Fehler werden nicht bemerkt

- **Symptom**: Fehler treten auf, werden aber nicht angezeigt und ignoriert

```ts
import { of, throwError } from 'rxjs';
import { mergeMap, catchError } from 'rxjs';

// ❌ Ohne Fehlerbehandlung werden Fehler unterdrückt
of(1, 2, 3)
  .pipe(
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Fehler'));
      }
      return of(value);
    })
  )
  .subscribe(); // Kein Error-Handler

// ✅ Angemessene Fehlerbehandlung
of(1, 2, 3)
  .pipe(
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Fehler'));
      }
      return of(value);
    }),
    catchError((error: unknown) => {
      console.error('🔴 Fehler abgefangen:', (error instanceof Error ? error.message : String(error)));
      return of(-1); // Fallback-Wert
    })
  )
  .subscribe({
    next: value => console.log('Wert:', value),
    error: error => console.error('🔴 Fehler in Subscription:', error)
  });

// Ausgabe:
// Wert: 1
// 🔴 Fehler abgefangen: Fehler
// Wert: -1
```

### Einrichtung eines globalen Error-Handlers

```ts
import { Observable } from 'rxjs';

// Alle unbehandelten Fehler abfangen
const originalCreate = Observable.create;

Observable.create = function(subscribe: any) {
  return originalCreate.call(this, (observer: any) => {
    try {
      return subscribe(observer);
    } catch (error) {
      console.error('🔴 Unbehandelter Fehler:', error);
      observer.error(error);
    }
  });
};
```

## Szenario 6: Anzahl der Retry-Versuche verfolgen

- **Symptom**: Der `retry`-Operator wird verwendet, aber es ist unklar, wie oft ein Retry durchgeführt wird

Bei automatischen Retries nach Fehlern wird das Debugging und Logging durch die Verfolgung der tatsächlichen Anzahl von Retry-Versuchen erleichtert.

### Grundlegendes Retry-Debugging

```ts
import { throwError, of, timer } from 'rxjs';
import { retry } from 'rxjs';

throwError(() => new Error('Temporärer Fehler'))
  .pipe(
    // Empfohlen ab RxJS 7.3+: retry({ count, delay })-Form
    retry({
      count: 2, // Maximal 2 Wiederholungen
      delay: (error, retryCount) => {
        console.log(`🔄 Retry ${retryCount}. Versuch`);
        return timer(1000);
      }
    })
  )
  .subscribe({
    next: value => console.log('✅ Erfolg:', value),
    error: error => {
      console.log('❌ Maximale Retry-Anzahl erreicht');
      console.log('🔴 Endgültiger Fehler:', error.message);
    }
  });

// Ausgabe:
// 🔄 Retry 1. Versuch
// 🔄 Retry 2. Versuch
// ❌ Maximale Retry-Anzahl erreicht
// 🔴 Endgültiger Fehler: Temporärer Fehler
```

> [!TIP]
> Detailliertere Implementierungsmuster für Retry-Debugging werden im Abschnitt "Retry-Debugging" unter [retry und catchError](/de/guide/error-handling/retry-catch#retry-debugging) erläutert.
> - Grundlegende Verfolgung mit tap error callback
> - Detailliertes Logging mit retryWhen
> - Exponentielles Backoff und Logging
> - RxJS 7.4+ retry Konfigurationsobjekt

## Zusammenfassung

Lösungsmethoden für häufige Debugging-Szenarien

- ✅ **Keine Werte fließen** → Überprüfung von vergessenen Subscriptions, Filterbedingungen
- ✅ **Unerwartete Werte** → Beachten Sie die Reihenfolge der Operatoren und gemeinsame Referenzen
- ✅ **Subscription wird nicht abgeschlossen** → Verwendung von `take` oder `takeUntil` für unendliche Streams
- ✅ **Memory Leak** → Automatisches Unsubscribe mit `takeUntil`-Pattern
- ✅ **Übersehene Fehler** → Implementierung angemessener Fehlerbehandlung
- ✅ **Retry-Verfolgung** → Logging mit `retryWhen` oder Konfigurationsobjekt

## Verwandte Seiten

- [Grundlegende Debug-Strategien](/de/guide/debugging/) - Verwendung von tap-Operator und Entwicklerwerkzeugen
- [Benutzerdefinierte Debug-Tools](/de/guide/debugging/custom-tools) - Benannte Streams, Debug-Operatoren
- [Performance-Debugging](/de/guide/debugging/performance) - Überwachung der Subscription-Anzahl, Speichernutzung
- [Fehlerbehandlung](/de/guide/error-handling/strategies) - Fehlerbehandlungsstrategien
