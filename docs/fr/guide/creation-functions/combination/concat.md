---
description: "Cette section explique comment combiner plusieurs Observables en séquence avec la fonction de création concat et comment l'utiliser pour l'exécution des étapes et l'affichage de l'interface utilisateur."
---

# concat - concaténer des flux en séquence

`concat` est une fonction de création qui **exécute séquentiellement** plusieurs Observables dans l'ordre spécifié.
L'Observable suivant est émis après que l'Observable précédent soit `complete`.

## Syntaxe de base et utilisation

```ts
import { concat, of, delay } from 'rxjs';

const obs1$ = of('A', 'B').pipe(delay(100));
const obs2$ = of('C', 'D').pipe(delay(100));

concat(obs1$, obs2$).subscribe(console.log);
// Sortie: A → B → C → D
```

- Une fois que tous les `obs1$` ont été émis, les `obs2$` commenceront à être émis.
- Le point clé est que les flux ne sont pas exécutés simultanément, mais "dans l'ordre".

[🌐 Documentation officielle RxJS - `concat`](https://rxjs.dev/api/index/function/concat)


## Modèles d'utilisation typiques

- **Traitement étape par étape** : Lorsque vous souhaitez passer à l'étape suivante une fois que l'étape précédente est terminée.
- **Requêtes API à ordre garanti** : Opérations asynchrones qui doivent être exécutées dans l'ordre.
- **Contrôle des événements de l'interface utilisateur** pour lesquels l'ordre est important, tels que les animations et les notifications.

## Exemples de code pratique (avec interface utilisateur)

Voici un exemple d'**affichage de messages de chargement et de listes de données dans un ordre séquentiel**.

```ts
import { concat, of, timer } from 'rxjs';
import { map, take } from 'rxjs';

// Créer une zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Exemple pratique de concat:</h3>';
document.body.appendChild(output);

// Flux de chargement
const loading$ = timer(0, 1000).pipe(
  map((count) => `⏳ Chargement... (${count + 1}s)`),
  take(3) // Émettre seulement pendant 3 secondes
);

// Flux de liste de données
const data$ = of('🍎 Pomme', '🍌 Banane', '🍇 Raisin');

// Concaténer et afficher dans l'ordre
concat(loading$, data$).subscribe((value) => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});
```

- Le message de chargement est d'abord affiché trois fois,
- puis la liste de données est affichée dans l'ordre.
- L'utilisation de **concat** permet d'obtenir facilement un affichage naturel "étape par étape".


## Opérateurs associés

- **[concatWith](/fr/guide/operators/combination/concatWith)** - Version opérateur Pipeable (utilisée dans le pipeline)
- **[concatMap](/fr/guide/operators/transformation/concatMap)** - mappe et concatène chaque valeur séquentiellement
- **[merge](/fr/guide/creation-functions/combination/merge)** - fonction de création de concaténation parallèle
