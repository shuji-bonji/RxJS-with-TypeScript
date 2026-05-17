---
description: Der throttleTime-Operator lässt innerhalb eines angegebenen Zeitintervalls nur den ersten Wert durch und ignoriert spätere Werte, wodurch hochfrequente Ereignisse effizient reduziert werden. Ideal zur Optimierung von Echtzeit-Ereignissen wie Scrollen oder Mausbewegungen.
---

# throttleTime - Erster Wert dann Limit

Der `throttleTime`-Operator lässt den zuerst ausgegebenen Wert durch und ignoriert nachfolgende Werte, die innerhalb des angegebenen Zeitintervalls ausgegeben werden.
Gibt nicht regelmäßig den neuesten Wert aus, sondern **lässt nur den zuerst empfangenen Wert durch und ignoriert danach für eine Zeit**.

Effektiv, wenn Sie Streams mit hoher Auslösefrequenz wie Scroll-Ereignisse oder Mausbewegungsereignisse reduzieren möchten.


## 🔰 Grundlegende Syntax und Verwendung

```ts
import { fromEvent } from 'rxjs';
import { throttleTime } from 'rxjs';

fromEvent(document, 'click')
  .pipe(throttleTime(2000))
  .subscribe(() => console.log('Geklickt!'));

```

- Empfängt nur das erste Click-Ereignis im 2-Sekunden-Intervall, spätere Klicks werden ignoriert.

[🌐 RxJS Offizielle Dokumentation - `throttleTime`](https://rxjs.dev/api/operators/throttleTime)

> [!WARNING] Hinweis für Produktionscode
> Das obige Beispiel lässt die Abmeldung von `fromEvent` zur Vereinfachung der Erklärung weg. Verwenden Sie in echtem Code `takeUntil(destroy$)`, `take(N)` oder `Subscription.unsubscribe()`, um den Lebenszyklus explizit zu verwalten. Details: [Schwierigkeiten überwinden: Lebenszyklus-Verwaltung](/de/guide/overcoming-difficulties/lifecycle-management.md)


## 💡 Typische Anwendungsmuster

- Ereignisbehandlungsoptimierung für Scrollen oder Mausbewegung
- Verhinderung von Mehrfachübermittlung durch Button-Mehrfachklick
- Reduzierung von Echtzeit-Datenstreams


## 🧠 Praktisches Codebeispiel (mit UI)

Zeigt Positionsinformationen alle 100 Millisekunden an, wenn Maus bewegt wird.

```ts
import { fromEvent } from 'rxjs';
import { throttleTime, map } from 'rxjs';

// Ausgabebereich erstellen
const container = document.createElement('div');
container.style.height = '200px';
container.style.border = '1px solid #ccc';
container.style.padding = '10px';
container.textContent = 'Bewegen Sie die Maus in diesem Bereich';
document.body.appendChild(container);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
document.body.appendChild(positionDisplay);

// Mausbewegungsereignis
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => ({
    x: event.clientX,
    y: event.clientY
  })),
  throttleTime(100)
).subscribe(position => {
  positionDisplay.textContent = `Mausposition: X=${position.x}, Y=${position.y}`;
});
```

- Begrenzt häufig ausgelöste Mausbewegungsereignisse auf alle 100ms und zeigt nur neueste Position an.
