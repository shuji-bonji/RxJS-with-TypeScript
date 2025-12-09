---
description: "Beschreibt, wie man mehrere Observables nacheinander mit der concat-Erstellungsfunktion verkettet. Da die erste Observable abgeschlossen wird, bevor die nächste beginnt, kann sie für Schrittausführung, sequenzielle UI-Anzeige, sequenzielle API-Aufrufe usw. verwendet werden."
---

# concat - Streams nacheinander verketten

`concat` ist eine Erstellungsfunktion, die **sequenziell** mehrere Observables in der angegebenen Reihenfolge ausführt.
Die Veröffentlichung der nächsten Observable beginnt, nachdem die vorherige Observable `complete` ist.

## Grundlegende Syntax und Verwendung

```ts
import { concat, of, delay } from 'rxjs';

const obs1$ = of('A', 'B').pipe(delay(100));
const obs2$ = of('C', 'D').pipe(delay(100));

concat(obs1$, obs2$).subscribe(console.log);
// Ausgabe: A → B → C → D
```

- Nachdem alle `obs1$` ausgegeben wurden, beginnt die Ausgabe von `obs2$`.
- Wichtig ist, dass die Streams nicht gleichzeitig, sondern "der Reihe nach" ausgeführt werden.

[🌐 Offizielle RxJS-Dokumentation - concat](https://rxjs.dev/api/index/function/concat)

## Typische Nutzungsmuster

- **Schrittweise Verarbeitung**: Wenn Sie mit dem nächsten Prozess fortfahren wollen, nachdem der vorherige abgeschlossen wurde
- **Reihenfolgegarantierte API-Anforderungen**: Wenn Sie eine Reihe von asynchronen Operationen nacheinander durchführen wollen
- **Kontrolle von UI-Ereignissen**, bei denen die Reihenfolge wichtig ist, z.B. Animationen und Benachrichtigungen

## Praktisches Code-Beispiel (mit UI)

Beispiel für die **Anzeige** von Lademeldungen und Datenlisten **der Reihe nach**.

```ts
import { concat, of, timer } from 'rxjs';
import { map, take } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>concat Praxisbeispiel:</h3>';
document.body.appendChild(output);

// Lade-Stream
const loading$ = timer(0, 1000).pipe(
  map((count) => `⏳ Laden... (${count + 1}s)`),
  take(3) // Nur 3 Sekunden lang ausgeben
);

// Datenlisten-Stream
const data$ = of('🍎 Apple', '🍌 Banana', '🍇 Grape');

// Mit concat der Reihe nach anzeigen
concat(loading$, data$).subscribe((value) => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});
```

- Die Lademeldung wird zunächst dreimal angezeigt,
- Die Datenliste wird dann der Reihe nach angezeigt.
- Mit **concat** kann eine natürliche "Schritt-für-Schritt"-Anzeige leicht erreicht werden.

## Verwandte Operatoren

- **[concatWith](/de/guide/operators/combination/concatWith)** - Pipeable Operator Version (wird in der Pipeline verwendet)
- **[concatMap](/de/guide/operators/transformation/concatMap)** - Jeden Wert sequentiell abbilden und zusammenführen
- **[merge](/de/guide/creation-functions/combination/merge)** - Parallel kombinieren (Erstellungsfunktion)
