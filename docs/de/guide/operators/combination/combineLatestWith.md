---
description: combineLatestWith ist ein RxJS-Kombinationsoperator, der die neuesten Werte des ursprünglichen Observables und anderer Observables kombiniert und ausgibt. Ideal für Echtzeit-Validierung von Formulareingaben, Synchronisierung mehrerer Zustände und Echtzeit-Aktualisierung von Berechnungsergebnissen, wenn Sie die neuesten Werte mehrerer abhängiger Streams kontinuierlich überwachen möchten. Als Pipeable Operator bequem für die Verwendung in Pipelines.
---

# combineLatestWith - Neueste kombinieren

Der `combineLatestWith`-Operator gibt **alle neuesten Werte zusammen** des ursprünglichen Observables und anderer angegebener Observables aus.
Jedes Mal, wenn von einem der Observables ein neuer Wert ausgegeben wird, wird ein Ergebnis mit allen neuesten Werten ausgegeben.
Dies ist die Pipeable Operator-Version der Creation Function `combineLatest`.

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

// Ausgabebeispiel:
// A0 + B0
// A1 + B0
// A2 + B0
// A2 + B1
```

- Kombinierte Werte werden erst ausgegeben, **nachdem jedes Observable mindestens einen Wert ausgegeben hat**.
- Jedes Mal, wenn von einem der beiden ein neuer Wert kommt, wird das neueste Paar erneut ausgegeben.

[🌐 RxJS Official Documentation - `combineLatestWith`](https://rxjs.dev/api/operators/combineLatestWith)


## 💡 Typische Anwendungsmuster

- **Echtzeit-Validierung von Formulareingaben**: Kontinuierliche Überwachung des neuesten Zustands mehrerer Felder
- **Synchronisierung mehrerer abhängiger Zustände**: Kombination von Konfigurationswerten und Benutzereingaben
- **Echtzeit-Aktualisierung von Berechnungsergebnissen**: Sofortige Berechnung von Ergebnissen aus mehreren Eingabewerten


## 🧠 Praktisches Codebeispiel (mit UI)

Ein Beispiel, das in Echtzeit den Gesamtbetrag aus Preis- und Mengeneingaben berechnet.

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>combineLatestWith Praxisbeispiel:</h3>';
document.body.appendChild(output);

// Eingabefelder erstellen
const priceInput = document.createElement('input');
priceInput.type = 'number';
priceInput.placeholder = 'Stückpreis';
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

// Observable für jede Eingabe
const price$ = fromEvent(priceInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(100)
);

const quantity$ = fromEvent(quantityInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(1)
);

// Neueste Werte kombinieren und berechnen
price$
  .pipe(
    combineLatestWith(quantity$),
    map(([price, quantity]) => price * quantity)
  )
  .subscribe((total) => {
    result.innerHTML = `<strong>Gesamtbetrag: €${total.toLocaleString()}</strong>`;
  });
```

- Bei Eingabe in einem der Felder wird **die Summe sofort aus den neuesten beiden Werten neu berechnet**.
- Durch Verwendung von `startWith()` kann von Anfang an ein kombiniertes Ergebnis erhalten werden.


## 🔄 Unterschied zur Creation Function `combineLatest`

### Grundlegender Unterschied

| | `combineLatest` (Creation Function) | `combineLatestWith` (Pipeable Operator) |
|:---|:---|:---|
| **Verwendungsort** | Als unabhängige Funktion | Innerhalb einer `.pipe()`-Kette |
| **Schreibweise** | `combineLatest([obs1$, obs2$])` | `obs1$.pipe(combineLatestWith(obs2$))` |
| **Erster Stream** | Alle gleichwertig behandelt | Als Hauptstream behandelt |
| **Rückgabewert** | Array `[val1, val2]` | Tupel `[val1, val2]` |
| **Vorteil** | Einfach und lesbar | Leicht mit anderen Operatoren kombinierbar |

### Konkrete Beispiele für die Auswahl

**Wenn nur einfache Kombination → Creation Function empfohlen**

```ts
import { combineLatest, of } from 'rxjs';

const firstName$ = of('Taro');
const lastName$ = of('Yamada');
const age$ = of(30);

// Einfach und lesbar
combineLatest([firstName$, lastName$, age$]).subscribe(([first, last, age]) => {
  console.log(`${last} ${first} (${age} Jahre)`);
});
// Ausgabe: Yamada Taro (30 Jahre)
```

**Wenn Transformation zum Hauptstream hinzugefügt werden soll → Pipeable Operator empfohlen**

```ts
import { fromEvent, interval } from 'rxjs';
import { combineLatestWith, map, startWith, debounceTime } from 'rxjs';

const searchInput = document.createElement('input');
searchInput.placeholder = 'Suche...';
document.body.appendChild(searchInput);

const categorySelect = document.createElement('select');
categorySelect.innerHTML = '<option>Alle</option><option>Bücher</option><option>DVDs</option>';
document.body.appendChild(categorySelect);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Hauptstream: Suchbegriff
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  debounceTime(300),  // 300ms nach Eingabe warten
  startWith('')
);

// Substream: Kategorieauswahl
const category$ = fromEvent(categorySelect, 'change').pipe(
  map(e => (e.target as HTMLSelectElement).value),
  startWith('Alle')
);

// ✅ Pipeable Operator-Version - vollständig in einer Pipeline
searchTerm$
  .pipe(
    map(term => term.toLowerCase()),  // In Kleinbuchstaben konvertieren
    combineLatestWith(category$),
    map(([term, category]) => ({
      term,
      category,
      timestamp: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(result => {
    output.textContent = `Suche: "${result.term}" Kategorie: ${result.category} [${result.timestamp}]`;
  });

// ❌ Creation Function-Version - wird umständlich
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
  output.textContent = `Suche: "${result.term}" Kategorie: ${result.category} [${result.timestamp}]`;
});
```

**Wenn mehrere Konfigurationswerte kombiniert werden**

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// Schieberegler erstellen
const redSlider = document.createElement('input');
redSlider.type = 'range';
redSlider.min = '0';
redSlider.max = '255';
redSlider.value = '255';
document.body.appendChild(document.createTextNode('Rot: '));
document.body.appendChild(redSlider);
document.body.appendChild(document.createElement('br'));

const greenSlider = document.createElement('input');
greenSlider.type = 'range';
greenSlider.min = '0';
greenSlider.max = '255';
greenSlider.value = '0';
document.body.appendChild(document.createTextNode('Grün: '));
document.body.appendChild(greenSlider);
document.body.appendChild(document.createElement('br'));

const blueSlider = document.createElement('input');
blueSlider.type = 'range';
blueSlider.min = '0';
blueSlider.max = '255';
blueSlider.value = '0';
document.body.appendChild(document.createTextNode('Blau: '));
document.body.appendChild(blueSlider);

const colorBox = document.createElement('div');
colorBox.style.width = '200px';
colorBox.style.height = '100px';
colorBox.style.marginTop = '10px';
colorBox.style.border = '1px solid #ccc';
document.body.appendChild(colorBox);

// Hauptstream: Rot
const red$ = fromEvent(redSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(255)
);

// ✅ Pipeable Operator-Version - Rot als Hauptstream, andere Farben kombinieren
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

### Zusammenfassung

- **`combineLatest`**: Optimal, wenn mehrere Streams einfach kombiniert werden sollen
- **`combineLatestWith`**: Optimal, wenn dem Hauptstream Transformationen oder Verarbeitungen hinzugefügt werden sollen, während die neuesten Werte anderer Streams kombiniert werden


## ⚠️ Wichtige Hinweise

### Keine Ausgabe bis Initialwerte vorhanden

Es werden keine Ergebnisse ausgegeben, bis alle Observables mindestens einen Wert ausgegeben haben.

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER)  // Observable, das keine Werte ausgibt
).subscribe(console.log);
// Keine Ausgabe (da NEVER keine Werte ausgibt)
```

In diesem Fall kann das Problem mit `startWith()` gelöst werden, indem ein Initialwert bereitgestellt wird.

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take, startWith } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER.pipe(startWith(null)))
).subscribe(console.log);
// Ausgabe: [0, null] → [1, null] → [2, null]
```

### Vorsicht bei häufiger Neuausgabe

Wenn einer der Streams häufig Werte ausgibt, werden auch die Ergebnisse häufig neu ausgegeben.

```ts
import { interval } from 'rxjs';
import { combineLatestWith } from 'rxjs';

// Stream, der alle 100ms ausgibt
const fast$ = interval(100);
const slow$ = interval(1000);

fast$.pipe(
  combineLatestWith(slow$)
).subscribe(console.log);
// Jedes Mal, wenn slow$ ausgibt, wird es mit dem neuesten Wert von fast$ kombiniert
// → Performance beachten
```

### Fehlerbehandlung

Wenn in einem der Observables ein Fehler auftritt, endet der gesamte Stream mit einem Fehler.

```ts
import { throwError, interval } from 'rxjs';
import { combineLatestWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  combineLatestWith(
    throwError(() => new Error('Fehler aufgetreten')).pipe(
      catchError(err => of('Wiederhergestellt'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error(err.message)
});
// Ausgabe: [0, 'Wiederhergestellt'] → [1, 'Wiederhergestellt']
```


## 📚 Verwandte Operatoren

- **[combineLatest](/de/guide/creation-functions/combination/combineLatest)** - Creation Function-Version
- **[withLatestFrom](/de/guide/operators/combination/withLatestFrom)** - Nur Hauptstream als Trigger
- **[zipWith](/de/guide/operators/combination/zipWith)** - Entsprechende Werte paaren
