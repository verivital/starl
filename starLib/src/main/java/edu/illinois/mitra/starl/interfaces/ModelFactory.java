package edu.illinois.mitra.starl.interfaces;

import edu.illinois.mitra.starl.models.Model;
import edu.illinois.mitra.starl.objects.ItemPosition;

@FunctionalInterface
public interface ModelFactory {
    Model create(ItemPosition pos);
}
