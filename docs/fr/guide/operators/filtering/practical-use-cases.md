---
description: "Explique les cas d'utilisation pratiques des opérateurs de filtrage RxJS (debounceTime, throttleTime, distinctUntilChanged, filter, etc.). Présente des patterns pratiques pour extraire uniquement les valeurs nécessaires des flux comme la recherche temps réel, le défilement infini, le contrôle d'événements haute fréquence et la suppression de doublons avec des exemples de code TypeScript. Apprenez des techniques d'implémentation utiles pour le traitement d'événements UI et l'optimisation des performances."
---

# Cas d'utilisation pratiques

## Filtrage de recherche temps réel pour les entrées utilisateur

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  debounceTime,
  distinctUntilChanged,
  filter,
} from 'rxjs';

// Construction de l'UI
const searchInput = document.createElement('input');
searchInput.placeholder = 'Entrer un terme de recherche (3 caractères minimum)';
document.body.appendChild(searchInput);

const resultsContainer = document.createElement('div');
document.body.appendChild(resultsContainer);

// Flux d'événements
fromEvent(searchInput, 'input')
  .pipe(
    map((event) => (event.target as HTMLInputElement).value.trim()),
    debounceTime(300),
    distinctUntilChanged(),
    filter((term) => term.length >= 3)
  )
  .subscribe((searchTerm) => {
    resultsContainer.innerHTML = `Démarrage de la recherche pour "${searchTerm}"...`;
  });

```

- **Traite uniquement les entrées confirmées à intervalles de 300ms**.
- La recherche n'est exécutée que si **3 caractères ou plus** sont saisis.
- Les entrées consécutives du **même mot** sont ignorées.


## Simulation de défilement infini

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  filter,
  throttleTime,
  distinctUntilChanged,
  scan,
} from 'rxjs';

// Construction de l'UI
const scrollArea = document.createElement('div');
scrollArea.style.height = '200px';
scrollArea.style.overflow = 'auto';
scrollArea.style.border = '1px solid #ccc';
document.body.appendChild(scrollArea);

const itemsList = document.createElement('div');
scrollArea.appendChild(itemsList);

// Ajout des données initiales
function addItems(page: number) {
  for (let i = 1; i <= 10; i++) {
    const item = document.createElement('div');
    item.textContent = `Élément ${(page - 1) * 10 + i}`;
    itemsList.appendChild(item);
  }
}
addItems(1);

// Flux d'événements de défilement
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

- Charge les éléments suivants quand la position de défilement dépasse **80%**.
- Charge automatiquement jusqu'à **5 pages**.
- **Réduit les événements de défilement** à **toutes les 200ms**.


## Guide de sélection des opérateurs de filtrage

| Objectif | Opérateur | Description |
|:---|:---|:---|
| Passer uniquement les données correspondant à une condition | `filter` | Le filtrage le plus basique |
| Récupérer uniquement les premiers éléments | `take`, `first` | Limitation du nombre d'éléments |
| Attendre la confirmation d'entrée | `debounceTime` | Idéal pour les formulaires |
| Traiter uniquement à intervalles réguliers | `throttleTime` | Applicable au défilement et redimensionnement |
| Ignorer les valeurs identiques consécutives | `distinctUntilChanged` | Éviter le retraitement inutile de données identiques |


## Résumé

- Les opérateurs de filtrage sont essentiels pour contrôler les flux de données.
- Non seulement utilisés seuls, ils deviennent encore plus puissants **en combinaison**.
- Directement liés à **l'efficacité et l'amélioration des performances** dans les applications événementielles et le développement UI.
