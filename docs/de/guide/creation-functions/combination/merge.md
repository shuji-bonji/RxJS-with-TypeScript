---
description: "Die merge-Erstellungsfunktion abonniert mehrere Observables gleichzeitig und führt ihre jeweiligen Werte zu einer Echtzeitausgabe zusammen. Sie kann für Gleichzeitigkeit, Integration mehrerer Ereignisquellen und Echtzeit-Updates verwendet werden."
---

# merge - Mehrere Streams gleichzeitig zusammenführen

`merge` abonniert mehrere Observables gleichzeitig und gibt die Werte aus, sobald sie von jeder Observable ausgegeben werden.

## Grundlegende Syntax und Verwendung

```ts
import { merge, interval } from 'rxjs';
import { map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `Stream 1: ${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `Stream 2: ${val}`),
  take(2)
);

merge(source1$, source2$).subscribe(console.log);
// Ausgabebeispiel:
// Stream 1: 0
// Stream 2: 0
// Stream 1: 1
// Stream 1: 2
// Stream 2: 1
```

- Alle Observables werden gleichzeitig abonniert, und die Werte fließen **in der Reihenfolge ein, in der sie ausgegeben werden**.
- Es gibt keine Garantie für die Reihenfolge und sie hängt **vom Ausgabezeitpunkt der einzelnen Observables** ab.

[🌐 Offizielle RxJS-Dokumentation - merge](https://rxjs.dev/api/index/function/merge)

## Typische Nutzungsmuster

- **Mehrere asynchrone Ereignisse zusammenführen** (z.B. Benutzereingaben und Backend-Benachrichtigungen)
- **Mehrere Datenströme in einen einzigen Stream zusammenfassen**
- **Parallele Informationsquellen kombinieren**, z.B. Echtzeit-Updates und Polling-Integration

## Praktisches Code-Beispiel (mit UI)

Kombiniert Klick- und Timer-Ereignisse in Echtzeit.

```ts
import { merge, fromEvent, timer } from 'rxjs';
import { map } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>merge Praxisbeispiel:</h3>';
document.body.appendChild(output);

// Button-Element erstellen
const button = document.createElement('button');
button.textContent = 'Klicken zum Auslösen';
document.body.appendChild(button);

// Klick-Stream
const click$ = fromEvent(button, 'click').pipe(
  map(() => '✅ Button-Klick erkannt')
);

// Timer-Stream
const timer$ = timer(3000, 3000).pipe(
  map((val) => `⏰ Timer-Ereignis (${val})`)
);

// Mit merge anzeigen
merge(click$, timer$).subscribe((value) => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});
```

- **Beim Klicken auf den Button wird sofort ein Ereignis erzeugt**,
- **Der Timer erzeugt alle 3 Sekunden ein sich wiederholendes Ereignis**.
- Erleben Sie die Möglichkeit, zwei verschiedene Arten von Observable in **Echtzeit zu kombinieren**.

> [!WARNING] Hinweis für Produktionscode
> Das obige Beispiel lässt die Abmeldung von `fromEvent` zur Vereinfachung der Erklärung weg. Verwenden Sie in echtem Code `takeUntil(destroy$)`, `take(N)` oder `Subscription.unsubscribe()`, um den Lebenszyklus explizit zu verwalten. Details: [Schwierigkeiten überwinden: Lebenszyklus-Verwaltung](/de/guide/overcoming-difficulties/lifecycle-management.md)

## Verwandte Operatoren

- **[mergeWith](/de/guide/operators/combination/mergeWith)** - Pipeable Operator Version (wird in der Pipeline verwendet)
- **[mergeMap](/de/guide/operators/transformation/mergeMap)** - Paralleles Mapping und Zusammenführen von Einzelwerten
- **[concat](/de/guide/creation-functions/combination/concat)** - Sequentielles Kombinieren (Erstellungsfunktion)
