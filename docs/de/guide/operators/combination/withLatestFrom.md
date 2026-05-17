---
description: "withLatestFrom ist ein Operator, der bei jeder Wertausgabe des Haupt-Observables den neuesten Wert eines anderen Streams kombiniert und ausgibt. Kann zur Erfassung des neuesten Zustands bei Formularübermittlung, Referenzierung von Eingabewerten beim Button-Klick und Kombination von Ereignissen und Zuständen verwendet werden."
---

# withLatestFrom - Mit neuestem Wert

Der `withLatestFrom`-Operator kombiniert **bei jeder Wertausgabe des Hauptstreams**
den **neuesten Wert** eines anderen Streams und gibt ihn aus.


## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval, fromEvent } from 'rxjs';
import { withLatestFrom, map, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
const timer$ = interval(1000);

clicks$
  .pipe(
    withLatestFrom(timer$),
    map(([click, timerValue]) => `Zähler beim Klick: ${timerValue}`)
  )
  .subscribe(console.log);

// Ausgabe:
// Zähler beim Klick: 1
// Zähler beim Klick: 2
// Zähler beim Klick: 2
// Zähler beim Klick: 5

```

- Das Haupt-Observable (hier Klick) wird zum Trigger,
- Der **neueste Wert** des Sub-Observables (hier Zähler) wird jedes Mal kombiniert und ausgegeben.

[🌐 RxJS Official Documentation - `withLatestFrom`](https://rxjs.dev/api/index/function/withLatestFrom)

> [!WARNING] Hinweis für Produktionscode
> Das obige Beispiel lässt die Abmeldung von `fromEvent` zur Vereinfachung der Erklärung weg. Verwenden Sie in echtem Code `takeUntil(destroy$)`, `take(N)` oder `Subscription.unsubscribe()`, um den Lebenszyklus explizit zu verwalten. Details: [Schwierigkeiten überwinden: Lebenszyklus-Verwaltung](/de/guide/overcoming-difficulties/lifecycle-management.md)


## 💡 Typische Anwendungsmuster

- **Neuesten Zustand bei Benutzeraktion abrufen**
- **Cache-Daten bei Anfrage referenzieren**
- **Ereignis-getriggerte Datenkombination**


## 🧠 Praktisches Codebeispiel (mit UI)

Ein Beispiel, das alle 2 Sekunden den neuesten Wert eines Eingabefelds abruft und anzeigt.

```ts
import { fromEvent, interval } from 'rxjs';
import { map, startWith, withLatestFrom } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'withLatestFrom Alle 2 Sekunden neueste Eingabe abrufen:';
document.body.appendChild(title);

// Eingabefeld erstellen
const nameInput = document.createElement('input');
nameInput.placeholder = 'Name eingeben';
document.body.appendChild(nameInput);

// Ausgabebereich erstellen
const output = document.createElement('div');
document.body.appendChild(output);

// Eingabe-Observable
const name$ = fromEvent(nameInput, 'input').pipe(
  map((e) => (e.target as HTMLInputElement).value),
  startWith('') // Zu Beginn leeren String ausgeben
);

// Timer (alle 2 Sekunden auslösen)
const timer$ = interval(2000);

// Bei jedem Timer-Auslösen den neuesten Eingabewert abrufen
timer$.pipe(withLatestFrom(name$)).subscribe(([_, name]) => {
  const item = document.createElement('div');
  item.textContent = `Alle 2 Sekunden abgerufen: Name: ${name}`;
  output.prepend(item);
});

```

- Während der Benutzer die Eingabe fortsetzt,
- Wird **alle 2 Sekunden der neueste Eingabeinhalt** abgerufen und angezeigt.
