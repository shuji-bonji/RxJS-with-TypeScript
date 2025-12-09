---
description: Der startWith-Operator ist ein Operator, der einen angegebenen Anfangswert einfügt, bevor das Observable Werte emittiert. Er eignet sich für Zustandsinitialisierung und anfängliche UI-Anzeige.
---

# startWith - Bereitstellung eines Anfangswerts

Der `startWith`-Operator ist ein Operator, der **einen angegebenen Anfangswert zuerst emittiert, bevor das Quell-Observable Werte emittiert**.
Er wird für Zustandsverwaltung, anfängliche Anzeige, Platzhalter-Werte usw. verwendet.


## 🔰 Grundlegende Syntax und Funktionsweise

```ts
import { of } from 'rxjs';
import { startWith } from 'rxjs';

of('B', 'C').pipe(
  startWith('A')
).subscribe(console.log);
// Ausgabe:
// A
// B
// C
```

Auf diese Weise fügt `startWith` zuerst `'A'` hinzu, gefolgt von den Werten des Quell-Observables.

[🌐 RxJS Offizielle Dokumentation - startWith](https://rxjs.dev/api/index/function/startWith)

## 💡 Typische Anwendungsfälle

Es ist praktisch, wenn Sie einen Anfangswert für einen Status oder Zähler festlegen möchten. Hier ist ein Beispiel für einen Zähler, der mit dem Anfangswert `100` beginnt.

```ts
import { interval } from 'rxjs';
import { startWith, scan, take } from 'rxjs';

interval(1000)
  .pipe(
    startWith(-1), // Zuerst -1 einfügen
    scan((acc, curr) => acc + 1, 100), // Inkrementieren ab Anfangswert 100
    take(10) // Insgesamt 10 Mal ausgeben
  )
  .subscribe(console.log);
// Ausgabe:
// 101
// 102
// 103
// 104
// 105
// 106
// 107
// 108
// 109
// 110
```


## 🧪 Praktisches Codebeispiel (mit UI)

```ts
import { interval } from 'rxjs';
import { startWith, scan, take } from 'rxjs';

// Ausgabebereich
const startWithOutput = document.createElement('div');
startWithOutput.innerHTML = '<h3>Beispiel für startWith:</h3>';
document.body.appendChild(startWithOutput);

// Zähler-Anzeigebereich
const counterDisplay = document.createElement('div');
counterDisplay.style.fontSize = '24px';
counterDisplay.style.fontWeight = 'bold';
counterDisplay.style.textAlign = 'center';
counterDisplay.style.padding = '20px';
counterDisplay.style.border = '1px solid #ddd';
counterDisplay.style.borderRadius = '5px';
counterDisplay.style.margin = '10px 0';
startWithOutput.appendChild(counterDisplay);

// Wertelisten-Anzeigebereich
const valuesList = document.createElement('div');
valuesList.style.marginTop = '10px';
startWithOutput.appendChild(valuesList);

// Zähler-Stream (jede Sekunde)
interval(1000)
  .pipe(
    // Zuerst ab 100 starten
    startWith(-1),
    // Jeden Wert zum vorherigen Wert +1 hinzufügen
    scan((acc, curr) => acc + 1, 100),
    // Nach 10 Mal beenden
    take(10)
  )
  .subscribe((count) => {
    // Zähler-Anzeige aktualisieren
    counterDisplay.textContent = count.toString();

    // Wert zur Liste hinzufügen
    const valueItem = document.createElement('div');

    if (count === 100) {
      valueItem.textContent = `Anfangswert: ${count} (mit startWith hinzugefügt)`;
      valueItem.style.color = 'blue';
    } else {
      valueItem.textContent = `Nächster Wert: ${count}`;
    }

    valuesList.appendChild(valueItem);
  });
```


## ✅ Zusammenfassung

- `startWith` ist praktisch, wenn Sie **zuerst einen festen Wert einfügen** möchten
- Wird häufig für Zustandsinitialisierung, UI-Platzhalter, anfängliche Formularanzeige usw. verwendet
- In Kombination mit `scan` oder `combineLatest` auch zur **Konstruktion einer Zustandsverwaltungsgrundlage** verwendbar
