---
description: Praktische use cases voor RxJS filteroperators (debounceTime, throttleTime, distinctUntilChanged, filter, etc.) worden uitgelegd. Leer praktische patronen om alleen de waarden die u nodig heeft uit streams te extraheren, zoals realtime zoeken, oneindig scrollen, het controleren van hoogfrequente gebeurtenissen, deduplicatie, etc., met TypeScript codevoorbeelden. U leert nuttige implementatietechnieken voor UI-gebeurtenisafhandeling en prestatieoptimalisatie.
---

# Praktische use cases

## Realtime zoekfiltering van gebruikersinvoer

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  debounceTime,
  distinctUntilChanged,
  filter,
} from 'rxjs';

// Bouw UI
const searchInput = document.createElement('input');
searchInput.placeholder = 'Voer zoekterm in (3+ tekens)';
document.body.appendChild(searchInput);

const resultsContainer = document.createElement('div');
document.body.appendChild(resultsContainer);

// Gebeurtenisstream
fromEvent(searchInput, 'input')
  .pipe(
    map((event) => (event.target as HTMLInputElement).value.trim()),
    debounceTime(300),
    distinctUntilChanged(),
    filter((term) => term.length >= 3)
  )
  .subscribe((searchTerm) => {
    resultsContainer.innerHTML = `Zoeken starten naar "${searchTerm}"...`;
  });

```

- **Verwerkt alleen bevestigde invoer** met intervallen van 300ms.
- **Zoekopdrachten worden alleen uitgevoerd wanneer 3 of meer tekens** zijn ingevoerd.
- **Opeenvolgende invoer van hetzelfde woord** wordt genegeerd.


## Oneindig scrollen simulatie

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  filter,
  throttleTime,
  distinctUntilChanged,
  scan,
} from 'rxjs';

// Bouw UI
const scrollArea = document.createElement('div');
scrollArea.style.height = '200px';
scrollArea.style.overflow = 'auto';
scrollArea.style.border = '1px solid #ccc';
document.body.appendChild(scrollArea);

const itemsList = document.createElement('div');
scrollArea.appendChild(itemsList);

// Voeg initiële data toe
function addItems(page: number) {
  for (let i = 1; i <= 10; i++) {
    const item = document.createElement('div');
    item.textContent = `Item ${(page - 1) * 10 + i}`;
    itemsList.appendChild(item);
  }
}
addItems(1);

// Scroll-gebeurtenisstream
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

- Wanneer de scrollpositie **80% of meer** bereikt, worden de volgende items geladen.
- **Automatisch laden tot 5 pagina's**.
- **Scroll-gebeurtenissen** worden uitgedund **elke 200ms**.


## Samenvatting van het kiezen van filteroperators

| Wat u wilt doen | Operator | Beschrijving |
|:---|:---|:---|
| Alleen data doorlaten die aan voorwaarde voldoet | `filter` | Meest basale filtering |
| Alleen eerste paar items ophalen | `take`, `first` | Beperk aantal opgehaalde items |
| Wachten tot invoer bevestigd is | `debounceTime` | Ideaal voor formulierinvoer |
| Alleen op vaste intervallen verwerken | `throttleTime` | Toepassen op scroll, resize, etc. |
| Opeenvolgende dezelfde waarden negeren | `distinctUntilChanged` | Voorkom verspillende herverwerking van identieke data |


## Samenvatting

- Filteroperators zijn essentieel voor het controleren van datastreams.
- Ze zijn niet alleen krachtig wanneer ze alleen worden gebruikt, maar nog krachtiger wanneer **gecombineerd**.
- Leiden direct tot **verbeterde efficiëntie en prestaties** in event-driven applicaties en UI-ontwikkeling.
