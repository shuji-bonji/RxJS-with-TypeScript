---
description: "combineLatestWith ist ein RxJS Kombinationsoperator, der das ursprüngliche Observable mit den neuesten Werten anderer Observables kombiniert. Er eignet sich ideal für Situationen, in denen Sie ständig die neuesten Werte mehrerer abhängiger Streams überwachen möchten, z. B. bei der Echtzeitvalidierung von Formulareingaben, der Synchronisierung mehrerer Zustände, der Echtzeitaktualisierung von Berechnungsergebnissen usw. Die pipe-fähige Operatorversion eignet sich für die Verwendung in Pipelines."
---

# combineLatestWith - kombiniert die letzten Werte

Der Operator "combineLatestWith" kombiniert alle **neuesten Werte** des ursprünglichen Observable und jedes andere angegebene Observable.
Immer wenn ein neuer Wert von einem der Observable ausgegeben wird, wird ein Ergebnis ausgegeben, das alle letzten Werte kombiniert.
Dies ist die Pipeable Operator-Version von `combineLatest` der Creation Function.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval } from 'rxjs';
import { combineLatestWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `A${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `B${val}`),
  take(2)
);

source1$
  .pipe(combineLatestWith(source2$))
  .subscribe(([val1, val2]) => {
    console.log(`${val1} + ${val2}`);
  });

// Beispiele für Ausgaben:
// A0 + B0
// A1 + B0
// A2 + B0
// A2 + B1
```

- Nachdem jedes Observable **mindestens einen Wert** ausgegeben hat, wird der kombinierte Wert ausgegeben.
- Immer wenn auf einer der beiden Seiten ein neuer Wert eingeht, wird das letzte Paar erneut ausgegeben.

[🌐 Offizielle RxJS Dokumentation - `combineLatestWith`](https://rxjs.dev/api/operators/combineLatestWith)

## 💡 Typisches Nutzungsmuster.

- **Echtzeitvalidierung von Formulareingaben**: ständige Überwachung des aktuellen Zustands mehrerer Felder
- **Synchronisierung mehrerer abhängiger Zustände**: Kombination von Konfigurationswerten und Benutzereingaben
- **Echtzeitaktualisierung von Berechnungsergebnissen**: sofortige Berechnung von Ergebnissen aus mehreren Eingabewerten

## 🧠 Praktische Code-Beispiele (mit UI)

Beispiel für die Berechnung des Gesamtbetrags in Echtzeit aus Preis- und Mengeneingaben.

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// Erstellung von Ausgabebereichen
const output = document.createElement('div');
output.innerHTML = '<h3>combineLatestWith Praktische Beispiele für:</h3>';
document.body.appendChild(output);

// Erstellung von Eingabefeldern
const priceInput = document.createElement('input');
priceInput.type = 'number';
priceInput.placeholder = 'Preis pro Einheit';
priceInput.value = '100';
document.body.appendChild(priceInput);

const quantityInput = document.createElement('input');
quantityInput.type = 'number';
quantityInput.placeholder = 'Menge';
quantityInput.value = '1';
document.body.appendChild(quantityInput);

// Ergebnisanzeigebereich
const result = document.createElement('div');
result.style.fontSize = '20px';
result.style.marginTop = '10px';
document.body.appendChild(result);

// der einzelnen EingabenObservable
const price$ = fromEvent(priceInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(100)
);

const quantity$ = fromEvent(quantityInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(1)
);

// Berechnet durch Kombination der letzten Werte
price$
  .pipe(
    combineLatestWith(quantity$),
    map(([price, quantity]) => price * quantity)
  )
  .subscribe((total) => {
    result.innerHTML = `<strong>Gesamtbetrag: ¥${total.toLocaleString()}</strong>`;
  });
```

- Wenn Sie eines der beiden Felder eingeben, wird die Gesamtsumme **sofort** aus den beiden letzten Werten neu berechnet.
- Die Funktion "startWith()" wird verwendet, um das kombinierte Ergebnis von Anfang an zu erhalten.

## 🔄 Unterschiede zur Creation Function `combineLatest`.

### Grundlegende Unterschiede.

```ts
import { interval } from 'rxjs';
import { combineLatestWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `A${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `B${val}`),
  take(2)
);

source1$
  .pipe(combineLatestWith(source2$))
  .subscribe(([val1, val2]) => {
    console.log(`${val1} + ${val2}`);
  });

// Beispiele für Ausgaben:
// A0 + B0
// A1 + B0
// A2 + B0
// A2 + B1
```

### Spezifische Beispiele für die Verwendung

**Wenn Sie nur einfache Kombinationen wünschen, ist die Creation Function die richtige Wahl**.


```ts
import { combineLatest, of } from 'rxjs';

const firstName$ = of('Taro');
const lastName$ = of('Yamada');
const age$ = of(30);

// Einfach und leicht zu lesen
combineLatest([firstName$, lastName$, age$]).subscribe(([first, last, age]) => {
  console.log(`${last} ${first}3 (${age}Alter)`);
});
// Ausgabe: Yamada Taro (30Alter)
```

**Wenn Sie dem Hauptstrom einen Konvertierungsprozess hinzufügen möchten, empfiehlt sich der Pipeable Operator**.

```ts
import { fromEvent, interval } from 'rxjs';
import { combineLatestWith, map, startWith, debounceTime } from 'rxjs';

const searchInput = document.createElement('input');
searchInput.placeholder = 'Suche nach...';
document.body.appendChild(searchInput);

const categorySelect = document.createElement('select');
categorySelect.innerHTML = '<option>Alle</option><option>Bücher</option><option>DVD</option>';
document.body.appendChild(categorySelect);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Mainstream: Schlüsselwörter suchen
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  debounceTime(300),  // Nach Eingabe300msWarten
  startWith('')
);

// Teilstrom: Auswahl der Kategorie
const category$ = fromEvent(categorySelect, 'change').pipe(
  map(e => (e.target as HTMLSelectElement).value),
  startWith('Alle')
);

// ✅ Pipeable OperatorAusgabe - Komplett in einer Pipeline
searchTerm$
  .pipe(
    map(term => term.toLowerCase()),  // In Kleinbuchstaben umgewandelt
    combineLatestWith(category$),
    map(([term, category]) => ({
      term,
      category,
      timestamp: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(result => {
    output.textContent = `Suche nach: "${result.term}" Kategorie: ${result.category} [${result.timestamp}]`;
  });

// ❌ Creation FunctionAusgabe - Überflüssig geworden
import { combineLatest } from 'rxjs';
combineLatest([
  searchTerm$.pipe(map(term => term.toLowerCase())),
  category$
]).pipe(
  map(([term, category]) => ({
    term,
    category,
    timestamp: new Date().toLocaleTimeString()
  }))
).subscribe(result => {
  output.textContent = `Suche nach: "${result.term}" Kategorie: ${result.category} [${result.timestamp}]`;
});
```

**Bei der Kombination mehrerer Konfigurationswerte**.

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// Schieberegler erstellen
const redSlider = document.createElement('input');
redSlider.type = 'range';
redSlider.min = '0';
redSlider.max = '255';
redSlider.value = '255';
document.body.appendChild(document.createTextNode('Red: '));
document.body.appendChild(redSlider);
document.body.appendChild(document.createElement('br'));

const greenSlider = document.createElement('input');
greenSlider.type = 'range';
greenSlider.min = '0';
greenSlider.max = '255';
greenSlider.value = '0';
document.body.appendChild(document.createTextNode('Green: '));
document.body.appendChild(greenSlider);
document.body.appendChild(document.createElement('br'));

const blueSlider = document.createElement('input');
blueSlider.type = 'range';
blueSlider.min = '0';
blueSlider.max = '255';
blueSlider.value = '0';
document.body.appendChild(document.createTextNode('Blue: '));
document.body.appendChild(blueSlider);

const colorBox = document.createElement('div');
colorBox.style.width = '200px';
colorBox.style.height = '100px';
colorBox.style.marginTop = '10px';
colorBox.style.border = '1px solid #ccc';
document.body.appendChild(colorBox);

// Mainstream: Red
const red$ = fromEvent(redSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(255)
);

// ✅ Pipeable OperatorAusgabe - RedKombiniere andere Farben als Hauptfarben
red$
  .pipe(
    combineLatestWith(
      fromEvent(greenSlider, 'input').pipe(
        map(e => Number((e.target as HTMLInputElement).value)),
        startWith(0)
      ),
      fromEvent(blueSlider, 'input').pipe(
        map(e => Number((e.target as HTMLInputElement).value)),
        startWith(0)
      )
    ),
    map(([r, g, b]) => `rgb(${r}, ${g}, ${b})`)
  )
  .subscribe(color => {
    colorBox.style.backgroundColor = color;
    colorBox.textContent = color;
    colorBox.style.display = 'flex';
    colorBox.style.alignItems = 'center';
    colorBox.style.justifyContent = 'center';
    colorBox.style.color = '#fff';
    colorBox.style.textShadow = '1px 1px 2px #000';
  });
```

### Zusammenfassung.

- **`combineLatest`**: ideal, wenn Sie einfach mehrere Datenströme kombinieren wollen
- **`combineLatestWith`**: ideal, wenn Sie die neuesten Werte aus anderen Streams kombinieren wollen, während Sie den Hauptstream transformieren oder verarbeiten

## ⚠️ Hinweise.

### Wird erst ausgegeben, wenn die ersten Werte verfügbar sind.

Es werden keine Ergebnisse ausgegeben, bis alle Observable mindestens einen Wert ausgegeben haben.

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER)  // Kein Wert ausgegebenObservable
).subscribe(console.log);
// Keine Ausgabe (weilNEVERKeine Ausgabe (weil)
```

Dies kann gelöst werden, indem man einen Anfangswert mit `startWith()` bereitstellt.

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take, startWith } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER.pipe(startWith(null)))
).subscribe(console.log);
// Ausgabe: [0, null] → [1, null] → [2, null]
```

### Vorsicht vor häufigen Neuauflagen.

Wenn ein Stream häufig Werte ausgibt, wird auch das Ergebnis häufig neu ausgegeben.

```ts
import { interval } from 'rxjs';
import { combineLatestWith } from 'rxjs';

// 100msStream ausgegeben jedes
const fast$ = interval(100);
const slow$ = interval(1000);

fast$.pipe(
  combineLatestWith(slow$)
).subscribe(console.log);
// slow$jedes Mal, wenn ein Wert ausgegeben wird,fast$ausgegeben wird, wird er mit dem letzten Wert von
// → Leistung muss beachtet werden
```

### Fehlerbehandlung.

Wenn in einem Observable ein Fehler auftritt, wird der gesamte Prozess mit einem Fehler beendet.

```ts
import { throwError, interval } from 'rxjs';
import { combineLatestWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  combineLatestWith(
    throwError(() => new Error('Fehler treten auf')).pipe(
      catchError((err: unknown) => of('Wiederherstellung'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error(err.message)
});
// Ausgabe: [0, 'Wiederherstellung'] → [1, 'Wiederherstellung']
```

## 📚 Verwandte Operatoren.

- **[combineLatest](/de/guide/creation-functions/combination/combineLatest)** - Creation Function Version.
- **[withLatestFrom](/de/guide/operators/combination/withLatestFrom)** - nur durch Mainstream ausgelöst
- **[zipWith](/de/guide/operators/combination/zipWith)** - Paart entsprechende Werte
