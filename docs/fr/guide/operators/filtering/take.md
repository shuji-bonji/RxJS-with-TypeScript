---
description: "L'opérateur take récupère uniquement le premier nombre spécifié de valeurs d'un flux Observable et ignore les valeurs suivantes, complétant automatiquement le flux. Utile lorsque vous souhaitez extraire uniquement les premiers éléments de données."
---

# take - Récupérer uniquement le premier nombre spécifié de valeurs

L'opérateur `take` récupère uniquement **le premier nombre spécifié** de valeurs d'un flux et ignore les valeurs suivantes.
Après complétion, le flux se termine automatiquement avec `complete`.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  take(3)
).subscribe(console.log);
// Sortie: 0, 1, 2
```

- Récupère uniquement les 3 premières valeurs.
- Après avoir récupéré 3 valeurs, l'Observable se termine automatiquement avec `complete`.

[🌐 Documentation officielle RxJS - `take`](https://rxjs.dev/api/operators/take)

## 💡 Patterns d'utilisation typiques

- Afficher ou enregistrer uniquement les premiers éléments dans l'UI ou les logs
- Souscription temporaire pour récupérer uniquement la première réponse
- Récupération limitée de données de test ou de démo

## 🧠 Exemple de code pratique (avec UI)

Récupère et affiche uniquement les 5 premières valeurs émises toutes les secondes.

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

// Création de la zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Exemple pratique de take :</h3>';
document.body.appendChild(output);

// Émet des valeurs toutes les secondes
const source$ = interval(1000);

// Récupère uniquement les 5 premières valeurs
source$.pipe(take(5)).subscribe({
  next: (value) => {
    const item = document.createElement('div');
    item.textContent = `Valeur : ${value}`;
    output.appendChild(item);
  },
  complete: () => {
    const complete = document.createElement('div');
    complete.textContent = 'Terminé';
    complete.style.fontWeight = 'bold';
    output.appendChild(complete);
  },
});

```

- Les 5 premières valeurs (`0`, `1`, `2`, `3`, `4`) sont affichées dans l'ordre,
- Puis le message « Terminé » s'affiche.
