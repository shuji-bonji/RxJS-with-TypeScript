---
description: Erklärt praktische Anwendungsfälle von RxJS-Filteroperatoren (debounceTime, throttleTime, distinctUntilChanged, filter usw.). Vorstellung praktischer Muster zur Extraktion nur benötigter Werte aus Streams, wie Echtzeit-Suche, Infinite Scroll, hochfrequente Ereignissteuerung und Duplikatsentfernung. Mit TypeScript-Codebeispielen zum Erlernen praktischer Implementierungstechniken für UI-Ereignisverarbeitung und Leistungsoptimierung.
---

# Praktische Anwendungsfälle

## Echtzeit-Such-Filterung bei Benutzereingabe

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  debounceTime,
  distinctUntilChanged,
  filter,
} from 'rxjs';

// UI-Aufbau
const searchInput = document.createElement('input');
searchInput.placeholder = 'Suchbegriff eingeben (mind. 3 Zeichen)';
document.body.appendChild(searchInput);

const resultsContainer = document.createElement('div');
document.body.appendChild(resultsContainer);

// Ereignisstream
fromEvent(searchInput, 'input')
  .pipe(
    map((event) => (event.target as HTMLInputElement).value.trim()),
    debounceTime(300),
    distinctUntilChanged(),
    filter((term) => term.length >= 3)
  )
  .subscribe((searchTerm) => {
    resultsContainer.innerHTML = `Suche nach „${searchTerm}" wird gestartet...`;
  });

```

- Verarbeitet nur **bestätigte Eingaben im 300ms-Intervall**.
- Suche wird nur ausgeführt, wenn **3 oder mehr Zeichen** eingegeben wurden.
- **Gleiche aufeinanderfolgende Wörter** werden ignoriert.


## Infinite-Scroll-Simulation

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  filter,
  throttleTime,
  distinctUntilChanged,
  scan,
} from 'rxjs';

// UI-Aufbau
const scrollArea = document.createElement('div');
scrollArea.style.height = '200px';
scrollArea.style.overflow = 'auto';
scrollArea.style.border = '1px solid #ccc';
document.body.appendChild(scrollArea);

const itemsList = document.createElement('div');
scrollArea.appendChild(itemsList);

// Initialdaten hinzufügen
function addItems(page: number) {
  for (let i = 1; i <= 10; i++) {
    const item = document.createElement('div');
    item.textContent = `Element ${(page - 1) * 10 + i}`;
    itemsList.appendChild(item);
  }
}
addItems(1);

// Scroll-Ereignisstream
fromEvent(scrollArea, 'scroll')
  .pipe(
    throttleTime(200),
    map(() => ({
      scrollTop: scrollArea.scrollTop,
      scrollHeight: scrollArea.scrollHeight,
      clientHeight: scrollArea.clientHeight,
    })),
    map(
      ({ scrollTop, scrollHeight, clientHeight }) =>
        (scrollTop + clientHeight) / scrollHeight
    ),
    distinctUntilChanged(),
    filter((ratio) => ratio > 0.8),
    scan((page) => page + 1, 1),
    filter((page) => page <= 5)
  )
  .subscribe((page) => {
    addItems(page);
  });

```

- Lädt nächste Elemente, wenn Scrollposition **über 80%** ist.
- Automatisches Laden **bis zu 5 Seiten**.
- **Scroll-Ereignisse** werden **alle 200ms** reduziert verarbeitet.


## Übersicht zur Auswahl von Filteroperatoren

| Gewünschte Aktion | Operator | Beschreibung |
|:---|:---|:---|
| Nur Daten durchlassen, die Bedingung erfüllen | `filter` | Grundlegendste Filterung |
| Nur erste paar abrufen | `take`, `first` | Begrenzung Anzahl Einträge |
| Auf Eingabebestätigung warten | `debounceTime` | Ideal für Formulareingabe |
| Nur in bestimmten Intervallen verarbeiten | `throttleTime` | Für Scrollen oder Größenänderung geeignet |
| Aufeinanderfolgende gleiche Werte ignorieren | `distinctUntilChanged` | Vermeidung unnötiger Neuverarbeitung gleicher Daten |


## Zusammenfassung

- Filteroperatoren sind unverzichtbar für Datenstrom-Steuerung.
- Werden durch **Kombination** noch leistungsfähiger, nicht nur einzeln.
- Direkter Bezug zu **Effizienzsteigerung und Leistungsverbesserung** bei ereignisgesteuerten Anwendungen und UI-Entwicklung.
