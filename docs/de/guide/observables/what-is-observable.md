---
description: "Observable ist ein Kernkonzept in RxJS, das einen Datenstrom repräsentiert, der im Laufe der Zeit auftritt. Wie es sich von Promise unterscheidet, wie man abonniert (subscribe) und abmeldet (unsubscribe), Cold und Hot Observable, TypeScript-Typdefinitionen im Detail."
---

# Was ist ein Observable?

[📘 RxJS Official: Observable](https://rxjs.dev/api/index/class/Observable)

Observable in RxJS ist ein Kernkonstrukt, das "einen Datenfluss (Stream) darstellt, der im Laufe der Zeit auftritt". Es wurde auf der Grundlage des Observer-Musters entwickelt und kann asynchrone und ereignisgesteuerte Verarbeitung auf einheitliche Weise handhaben.

## Rolle von Observable

Ein Observable fungiert als "Datenproduzent", der mehrere Werte im Laufe der Zeit veröffentlicht. Im Gegensatz dazu agiert ein Observer als "Konsument" und abonniert Werte über `subscribe()`.

Im folgenden Beispiel wird ein **Observable (Produzent)** namens `observable$` erstellt und der **Observer (Konsument)** abonniert und empfängt Werte.

```ts
import { Observable } from 'rxjs';

// Erstellen eines Observable (Produzent)
const observable$ = new Observable<number>(subscriber => {
  // Auszuführende Logik bei Abonnement
  subscriber.next(1);
  subscriber.next(2);
  subscriber.complete();
});

// Observer (Konsument) abonniert
observable$.subscribe({
  next: value => console.log('Nächster Wert:', value),
  error: err => console.error('Fehler:', err),
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// Nächster Wert: 1
// Nächster Wert: 2
// Abgeschlossen
```

> [!NOTE]
> Die Funktion, die als Argument an `new Observable(function)` übergeben wird, definiert die **Logik, die ausgeführt wird, wenn das Observable abonniert wird**. Die Funktion selbst ist nicht der Produzent, sondern das Observable als Ganzes.

## Art der Benachrichtigung

Das Observable sendet die folgenden drei Arten von Benachrichtigungen an den Observer.

- `next`: Benachrichtigung über einen Wert
- `error`: Benachrichtigung bei einem Fehler (es werden keine weiteren Benachrichtigungen gesendet)
- `complete`: Benachrichtigung über den erfolgreichen Abschluss

Weitere Informationen finden Sie im Abschnitt [Observer im "Observable-Lebenszyklus"](./observable-lifecycle.md#_2-observer-observer).

## Unterschied zwischen Observable und Promise

| Feature | Observable | Promise |
|---|---|---|
| Mehrere Werte | ◯ | × (nur einer) |
| Abbrechbar | ◯ (`unsubscribe()`) | × |
| Verzögerte Ausführung | ◯ | ◯ |
| Synchron/Asynchron | Beides | Nur asynchron |

Der größte Unterschied zwischen Observable und Promise besteht darin, ob es mehrere Werte verarbeiten kann und ob es in der Mitte abgebrochen werden kann.
Promise eignet sich für eine einmalige asynchrone Verarbeitung, während Observable seine Stärken bei "kontinuierlich auftretenden asynchronen Daten" wie Ereignisströmen hat.

Observable ist auch im Hinblick auf die Ressourcenverwaltung wichtig, z. B. um Speicherlecks zu verhindern und unnötige Kommunikation zu unterbinden, da Abonnements mitten in einem Prozess durch `unsubscribe()` abgebrochen werden können.

Promise hingegen ist in der Standard-API weit verbreitet und kann in Kombination mit `async/await` auf intuitive Weise geschrieben werden. Je nach Anwendung ist es wünschenswert, beide zu verwenden.

## Unterscheidung zwischen Cold und Hot

In RxJS gibt es zwei Arten von Observable: "Cold" und "Hot".

- **Cold Observable**: Jeder Abonnent hat seinen eigenen Datenstrom, der mit der Ausführung beginnt, wenn er abonniert wird. (z.B. `of()`, `from()`, `fromEvent()`, `ajax()`)
- **Hot Observable**: Abonnenten teilen sich denselben Datenstrom und die Daten fließen weiter, unabhängig davon, ob sie abonniert sind oder nicht. (z.B. `Subject`, mit `share()` multicast Observable)

Diese Unterscheidung hat erhebliche Auswirkungen auf die gemeinsame Nutzung von Daten und die Ressourceneffizienz.
Für weitere Informationen siehe den Abschnitt ["Cold Observable und Hot Observable"](./cold-and-hot-observables.md).

## Observable und Pipeline

Der eigentliche Wert einer Observable ergibt sich aus der Kombination mit einem Operator unter Verwendung der Methode `pipe()`.

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5);
numbers$.pipe(
  filter(n => n % 2 === 0), // nur gerade Zahlen passieren
  map(n => n * 10)          // Konvertiert in 10-fach
).subscribe(value => console.log(value));
// Ausgabe: 20, 40
```

## Observable-Lebenszyklus

Ein Observable hat den folgenden Lebenszyklus:

1. **Erstellung** - Erstellung einer Observable-Instanz
2. **Subscribe** - Beginn des Empfangs von Daten durch `subscribe()`
3. **Ausführung** - Veröffentlichen von Daten (`next`), Fehler (`error`) oder Abschluss (`complete`)
4. **Unsubscribe** - Beenden des Abonnements durch `unsubscribe()`

Es ist wichtig, Observable-Abonnements, die nicht mehr benötigt werden, abzubestellen, um Ressourcenlecks zu vermeiden.
Für weitere Informationen siehe den Abschnitt ["Observable-Lebenszyklus"](./observable-lifecycle.md).

## Wo Observable zu verwenden ist

- UI-Ereignisse (z.B. Klicks, Scrollen, Tastaturaktionen)
- HTTP-Anfragen
- Zeitbasierte Verarbeitung (Intervalle und Timer)
- WebSockets und Echtzeitkommunikation
- Verwaltung des Anwendungsstatus

## Zusammenfassung

Observable ist die Grundlage für den flexiblen und einheitlichen Umgang mit asynchronen Daten. Als zentrales Konzept in ReactiveX (RxJS) bietet es eine übersichtliche Darstellung komplexer asynchroner Verarbeitungs- und Ereignisströme.
